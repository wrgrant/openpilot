"""Off-loop Odyssey generations. The loader never mutates a running module's globals."""
import ast
import hashlib
import functools
import inspect
import importlib
import importlib.abc
import importlib.util
import json
import os
import platform
import tempfile
from pathlib import Path
import sys
import threading
import types
import uuid

PREFIX = 'openpilot.sunnypilot.selfdrive.controls.lib.'
LIB = Path(__file__).parent
RELOAD_DIR = Path(os.environ.get('ODYSSEY_RELOAD_DIR', '/dev/shm/odyssey_reload' if platform.system() != 'Darwin'
                                     else str(Path(tempfile.gettempdir()) / 'odyssey_reload')))


def scope(folder=LIB):
  """Return dependency order, including same-folder helpers, excluding this bootstrap."""
  names = {p.stem for p in folder.glob('odyssey_*.py')} - {'odyssey_reload'}
  ordered, visiting = [], set()
  def visit(name):
    if name in ordered:
      return
    if name in visiting:
      raise ValueError(f'cyclic reload dependency: {name}')
    visiting.add(name)
    for node in ast.walk(ast.parse((folder / f'{name}.py').read_text())):
      if isinstance(node, ast.ImportFrom) and node.module and node.module.startswith(PREFIX):
        dependency = node.module.removeprefix(PREFIX)
        if '.' not in dependency and (folder / f'{dependency}.py').exists() and dependency != 'odyssey_reload':
          visit(dependency)
    visiting.remove(name)
    ordered.append(name)
  for name in sorted(names):
    visit(name)
  return ordered


class Generation(importlib.abc.MetaPathFinder, importlib.abc.Loader):
  def __init__(self):
    self.prefix = f'_odyssey_{uuid.uuid4().hex}.'
    self.sources = {name: (LIB / f'{name}.py').read_bytes() for name in scope()}
    self.hashes = {name: hashlib.sha256(data).hexdigest() for name, data in self.sources.items()}
    package = types.ModuleType(self.prefix[:-1])
    package.__path__ = []
    sys.modules[package.__name__] = package

  def find_spec(self, fullname, path=None, target=None):
    if fullname.startswith(self.prefix) and fullname.removeprefix(self.prefix) in self.sources:
      return importlib.util.spec_from_loader(fullname, self)
    return None

  def create_module(self, spec):
    return None

  def exec_module(self, module):
    name = module.__name__.removeprefix(self.prefix)
    # Redirect even function-local imports into this generation. External controllers keep their original helpers.
    source = self.sources[name].decode().replace(PREFIX, self.prefix)
    exec(compile(source, str(LIB / f'{name}.py'), 'exec'), module.__dict__)

  def load(self):
    sys.meta_path.insert(0, self)
    try:
      self.modules = {name: importlib.reload(importlib.import_module(self.prefix + name)) for name in self.sources}
      return self.modules
    except Exception:
      self.close()
      raise

  def close(self):
    if self in sys.meta_path:
      sys.meta_path.remove(self)
    for name in list(sys.modules):
      if name == self.prefix[:-1] or name.startswith(self.prefix):
        del sys.modules[name]


class Bundle:
  def __init__(self, owner):
    self.generation = Generation()
    try:
      modules = self.generation.load()
      self.steer = modules['odyssey_steer'].OdysseySteer(owner.params) if owner.odyssey_steer is not None else None
      self.lane = modules['odyssey_lane'].LaneCentering(owner.params)
      # Shells are prepared without starting workers or resetting drive params. State binds at the swap frame.
      self.ab = modules['odyssey_ab'].LiveAB.__new__(modules['odyssey_ab'].LiveAB)
      self.pulse = None
      self.controller = None
      if owner.odyssey_pulse is not None:
        self.pulse = modules['odyssey_pulse'].PulseRuntime.__new__(modules['odyssey_pulse'].PulseRuntime)
        self.controller = modules['odyssey_pulse'].Pulse()
        self.plan = self.controller.plan
        self.extra = modules['odyssey_pulse'].parse_queue(owner.params.get('OdysseyPulseQueue') or '[]')
    except Exception as exc:
      exc.reload_hashes = self.generation.hashes
      self.generation.close()
      raise

  def swap(self, owner):
    old = (owner.odyssey_ab, owner.odyssey_pulse)
    self.ab.__dict__ = owner.odyssey_ab.__dict__
    if self.pulse is not None:
      self.pulse.__dict__ = owner.odyssey_pulse.__dict__
      self.controller.__dict__ = owner.odyssey_pulse.controller.__dict__
      self.controller.plan = self.plan
      self.pulse.extra = self.controller.extra = self.extra
      self.pulse.controller = self.controller
    # Preserve the current manual selection until the replacement worker's first poll.
    previous_config = sys.modules[owner.odyssey_ab.__class__.__module__]._config
    self.generation.modules['odyssey_ab']._config = previous_config
    self.generation.modules['odyssey_ab']._model_arms = sys.modules[owner.odyssey_ab.__class__.__module__]._model_arms
    if getattr(owner, 'odyssey_reload', None) is not None:
      owner.odyssey_reload.modules = self.generation.modules
    owner.odyssey_steer, owner.odyssey_lane = self.steer, self.lane
    owner.odyssey_ab, owner.odyssey_pulse = self.ab, self.pulse
    return old

  def retire(self, old):
    for runtime, replacement, attr in ((old[0], self.ab, 'thread'), (old[1], self.pulse, 'worker')):
      if runtime is None:
        continue
      runtime.close()
      if getattr(runtime, attr).is_alive():
        raise RuntimeError(f'{attr} did not stop')
      replacement.stop = threading.Event()
      if attr == 'worker':
        replacement.resuming = True
      worker = threading.Thread(target=replacement._worker, daemon=True, name=f'odyssey-{attr}')
      setattr(replacement, attr, worker)
      worker.start()


class ReloadManager:
  def __init__(self, owner, directory=RELOAD_DIR, prepare=None, report=None):
    self.owner, self.directory = owner, directory
    self.module_names = scope()
    self.modules = {name: importlib.import_module(PREFIX + name) for name in scope()} if prepare is None else {}
    if prepare is None:
      self._bridge_helpers()
    self.prepare = prepare or (lambda stamp: Bundle(owner))
    self.report = report or self._report
    self.current = self.pending = self.applied = None
    self.stop = threading.Event()
    self.thread = threading.Thread(target=self._worker, daemon=True, name='odyssey-reload')

  def _bridge_helpers(self):
    # Controllers/NNLC outside the reload scope import functions directly. Install stable forwarders once,
    # while parked at process startup; each future activation changes only the module-table reference.
    def forward(name, function):
      @functools.wraps(function)
      def call(*args, **kwargs):
        return getattr(self.modules[name], function.__name__)(*args, **kwargs)
      return call
    for name, module in list(sys.modules.items()):
      if module is None or not name.startswith('openpilot.') or name.startswith(PREFIX):
        continue
      for attribute, value in list(vars(module).items()):
        if inspect.isfunction(value) and value.__module__.startswith(PREFIX):
          dependency = value.__module__.removeprefix(PREFIX)
          if dependency in self.modules:
            setattr(module, attribute, forward(dependency, value))

  def start(self):
    self.directory.mkdir(parents=True, exist_ok=True)
    self.thread.start()

  def close(self):
    self.stop.set()
    self.thread.join(timeout=2)

  def _report(self, result):
    from openpilot.common.swaglog import cloudlog
    from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_ab import mem_params
    cloudlog.event('odyssey_reload', **result)
    path = self.directory / 'result.tmp'
    path.write_text(json.dumps(result))
    path.replace(self.directory / 'result')
    note = f"code reloaded {result['stamp'][:12]}" if result['status'] == 'ok' else f"reload {result['status']}"
    mem_params(self.owner.params).put('OdysseyAgentNote', note + '\n' + result.get('message', '')[:90])

  def _worker(self):
    seen = None
    while not self.stop.wait(.05):
      try:
        if self.applied is not None:
          bundle, old, result, previous = self.applied
          if hasattr(bundle, 'retire'):
            bundle.retire(old)
          if previous is not None and hasattr(previous, 'generation'):
            previous.generation.close()
          self.applied = None
          self.report({**result, 'status': 'ok', 'message': 'reload applied'})
        if self.pending is not None:
          continue
        path = self.directory / 'request'
        stamp = path.read_text().strip() if path.exists() else ''
        if not stamp or stamp == seen:
          continue
        seen = stamp
        result = {'stamp': stamp, 'modules': self.module_names, 'hashes': {}, 'status': 'loading'}
        try:
          result['hashes'] = {name: hashlib.sha256((LIB / f'{name}.py').read_bytes()).hexdigest() for name in self.module_names}
          bundle = self.prepare(stamp)
          if hasattr(bundle, 'generation'):
            result['hashes'] = bundle.generation.hashes
            result['modules'] = list(bundle.generation.sources)
          self.pending = (bundle, result)
          self.report({**result, 'status': 'ready', 'message': 'waiting for safe frame'})
        except Exception as exc:
          self.report({**result, 'hashes': getattr(exc, 'reload_hashes', result['hashes']), 'status': 'error', 'message': f'{type(exc).__name__}: {exc}'})
      except Exception as exc:
        self.applied = None
        try:
          self.report({'stamp': seen or '', 'status': 'error', 'message': str(exc), 'modules': self.module_names})
        except Exception:
          pass  # reporting must never escape into controls

  def poll(self, lat_active):
    if self.pending is None or self.applied is not None:
      return False
    bundle, result = self.pending
    pulse = self.owner.odyssey_pulse
    if pulse is not None and pulse.controller.in_shot:
      return False
    if lat_active and not result['stamp'].endswith(':now'):
      return False
    old = bundle.swap(self.owner) if hasattr(bundle, 'swap') else None
    previous, self.current = self.current, bundle
    self.pending = None
    self.applied = (bundle, old, result, previous)
    return True
