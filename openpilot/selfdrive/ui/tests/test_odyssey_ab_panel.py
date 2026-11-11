"""Render the actual raylib widgets at comma 3X size, with synthetic subscribed state."""
import json
import os
from pathlib import Path
from types import SimpleNamespace
import time
import unittest
from unittest.mock import patch

from PIL import Image
import raylib_nb as rl
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app, MousePos
from openpilot.sunnypilot.selfdrive.controls.lib import odyssey_ab as ab


class TestOdysseyPanel(OpenpilotTestCase):
  def test_render_and_tap(self):
    output = Path(os.environ.get('ABUI_SCREENSHOTS', '/tmp/abui-screenshots'))
    output.mkdir(parents=True, exist_ok=True)
    gui_app._width, gui_app._height = 2160, 1080
    gui_app._scale = 1.
    gui_app._scaled_width, gui_app._scaled_height = 2160, 1080
    gui_app.init_window('Odyssey A/B test', fps=20)
    try:
      from openpilot.selfdrive.ui.sunnypilot.onroad.hud_renderer import HudRendererSP
      params = Params()
      with patch.object(ui_state, 'params', params):
        ui_state.update_params()
        sm = ui_state.sm
        msgs = []
        for service in ('carState', 'carControl', 'carOutput', 'controlsState', 'modelV2', 'vehicleParameters'):
          msg = messaging.new_message(service)
          if service == 'carState':
            msg.carState.vEgo = 25.
            msg.carState.vEgoCluster = 25.
            msg.carState.steeringAngleDeg = 2.
          elif service == 'carControl':
            msg.carControl.latActive = True
          elif service == 'carOutput':
            msg.carOutput.actuatorsOutput.torque = .75
          elif service == 'controlsState':
            msg.controlsState.lateralControlState.init('torqueState')
          elif service == 'modelV2':
            msg.modelV2.velocity.x = [25.] * 33
            msg.modelV2.orientationRate.z = [.12] * 33
          msgs.append(msg)
        sm.update_msgs(time.monotonic(), msgs)
        ui_state.started_frame = 0
        ui_state.status = UIStatus.ENGAGED
        ui_state.developer_ui = 3
        ui_state.torque_bar = True
        texture = rl.load_render_texture(2160, 1080)
        hud = HudRendererSP()
        panel = hud.odyssey_ab_panel
        for name in ('none', 'bump', 'counting', 'blinker', 'target', 'torque'):
          exp, arm = ('bump', 'kick') if name == 'bump' else ('lane', 'off')
          params.put('OdysseyABExperiment', '' if name == 'none' else exp, block=True)
          params.put_bool('OdysseyABManual', True, block=True)
          params.put('OdysseyABArm', arm, block=True)
          totals = ab.validate_totals('{}')
          if name == 'target':
            totals['lane']['off'] = {'straight': 300., 'curve': 60.}
          params.put('OdysseyABTotals', json.dumps(totals), block=True)
          status = {'experiment': exp, 'arm': arm, 'manual': True, 'good': name != 'blinker',
                        'reason': 'blinker' if name == 'blinker' else 'counting', 'kind': 'straight', 'mono_s': time.monotonic()}
          params.put('OdysseyABStatus', json.dumps(status), block=True)
          panel.last_read = 0
          # Real HUD render methods with a neutral road background; no camera or van is required.
          for _ in range(15):
            rl.begin_texture_mode(texture)
            rl.clear_background(rl.Color(38, 44, 52, 255))
            rect = rl.Rectangle(0, 0, 2160, 1080)
            hud.speed_renderer.update()
            hud._draw_current_speed(rect)
            hud.odyssey_ab_panel.render_in(rect)
            hud._torque_bar.render(rl.Rectangle(0, 0, 2160, 1020))
            hud.developer_ui.render(rect)
            rl.end_texture_mode()
          image = rl.load_image_from_texture(texture.texture)
          Image.frombytes('RGBA', (2160, 1080), image.to_bytes(2160*1080*4)).transpose(Image.Transpose.FLIP_TOP_BOTTOM).save(output / f'{name}.png')
          rl.unload_image(image)
          self.assertEqual(panel.e, '' if name == 'none' else exp)
          self.assertEqual(panel.contains(MousePos(100, 400)), name != 'none', 'the road view leaves touches on a visible panel alone')
        panel._handle_mouse_release(MousePos(100, 400))
        time.sleep(.05)
        self.assertEqual(params.get('OdysseyABArm'), 'on')
        # The worker's atomic snapshot drives the same selector the real lane controller calls.
        worker = ab.LiveAB(params)
        self.addCleanup(worker.close)
        deadline = time.monotonic() + .5
        while not ab.manual_experiment('lane') and time.monotonic() < deadline:
          time.sleep(.005)
        from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_lane import LaneCentering
        self.assertTrue(LaneCentering(params).active_arm(0))
        cs = SimpleNamespace(vEgo=25., steeringPressed=False, leftBlinker=False, rightBlinker=False)
        md = SimpleNamespace(laneLineProbs=[1.]*4, meta=SimpleNamespace(laneChangeState=0))
        for i in range(602):
          worker.update(i*.01, cs, md, True, 0.)
        self.assertTrue(worker.status['good'])
        worker.close()
        for name, reason in (('wait', 'waiting for a straight'), ('lead-in', 'lead-in'), ('pulse', 'pulsing'),
                             ('tail', 'tail'), ('recover', 're-centring'), ('paused', "you're steering"), ('done', 'done'),
                             ('manual', 'waiting for fire'), ('burst', 'pulsing'), ('auto', 'waiting for a straight')):
          params.put('OdysseyABExperiment', 'pulse', block=True)
          params.put('OdysseyABStatus', json.dumps({'experiment': 'pulse', 'mono_s': time.monotonic(),
                     'state': {'manual': 'wait', 'auto': 'wait', 'burst': 'pulse'}.get(name, name),
                     'reason': reason, 'finished': 7, 'total': 78, 'band': '45-70',
                     'mode': name if name in ('manual', 'burst', 'auto') else 'manual', 'burst': 5,
                     'series_index': 2, 'series_total': 5 if name == 'burst' else 0,
                     'next_shot': 'square', 'last_result': 'pair 6% 0.1s: wheel +0.30°', 'paused': name == 'paused',
                     'wheel': 920 if name == 'tail' else 140, 'wheel_limit': 800, 'ignore_wheel': name == 'tail'}), block=True)
          panel.last_read = 0
          rl.begin_texture_mode(texture)
          rl.clear_background(rl.Color(38, 44, 52, 255))
          hud._draw_current_speed(rect)
          hud._draw_set_speed(rect)
          panel.render_in(rl.Rectangle(0, 0, 2160, 1080))
          rl.end_texture_mode()
          image = rl.load_image_from_texture(texture.texture)
          Image.frombytes('RGBA', (2160, 1080), image.to_bytes(2160*1080*4)).transpose(Image.Transpose.FLIP_TOP_BOTTOM).save(output / f'pulse_{name}.png')
          rl.unload_image(image)
          self.assertFalse(panel.broken)
          if name in ('manual', 'burst', 'auto'):
            self.assertIn({'manual': 'Tap to fire: square', 'burst': 'burst 2 of 5', 'auto': 'auto • tap to fire'}[name], panel.drawn)
          from openpilot.selfdrive.ui.sunnypilot.onroad.odyssey_ab_panel import HEAD_H, PAUSE_H, FOOT_H
          self.assertGreaterEqual(min(HEAD_H, PAUSE_H, FOOT_H), 200)
          self.assertEqual(panel.shown.height - HEAD_H - PAUSE_H - FOOT_H, 400)
        params.put_bool('OdysseyPulsePaused', False, block=True)
        top, bottom = panel.shown.y, panel.shown.y + panel.shown.height
        panel._handle_mouse_release(MousePos(100, top + 550))  # middle fires while running
        time.sleep(.05)
        self.assertTrue(ab.mem_params(params).get('OdysseyPulseFire'))
        self.assertFalse(params.get_bool('OdysseyPulsePaused'))
        self.assertIn('auto • tap to fire', panel.drawn)
        panel._handle_mouse_release(MousePos(100, top + 300))  # status heading pauses
        time.sleep(.05)
        self.assertTrue(params.get_bool('OdysseyPulsePaused'))
        panel._handle_mouse_release(MousePos(100, bottom - 40))  # the bottom strip: ignore the wheel sensor, and back
        time.sleep(.05)
        self.assertTrue(params.get_bool('OdysseyPulseIgnoreWheel'))
        self.assertTrue(params.get_bool('OdysseyPulsePaused'), 'the bottom strip does not also start the test')
        panel._handle_mouse_release(MousePos(100, bottom - 40))
        time.sleep(.05)
        self.assertFalse(params.get_bool('OdysseyPulseIgnoreWheel'))
        seen = []
        for _ in range(3):  # the top strip: pulse -> bump -> lane -> pulse, each arriving safe
          panel._handle_mouse_release(MousePos(100, top + 20))
          time.sleep(.05)
          seen.append((params.get('OdysseyABExperiment'), params.get('OdysseyABArm') or '', params.get_bool('OdysseyABManual')))
          panel.e = seen[-1][0]
        self.assertEqual(seen, [('bump', 'off', True), ('lane', 'off', True), ('pulse', '', True)])
        self.assertTrue(params.get_bool('OdysseyPulsePaused'), 'the pulse test arrives paused')
        from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.honda import HondaSettings
        params.put_bool('OdysseyPulsePaused', False, block=True)
        HondaSettings._select_experiment('pulse')
        time.sleep(.05)
        self.assertFalse(params.get('OdysseyABArm'), 'pulse has no arms')
        self.assertTrue(params.get_bool('OdysseyPulsePaused'))

    finally:
      gui_app.close()


if __name__ == '__main__':
  unittest.main()
