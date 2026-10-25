"""Reference geometry and synthetic recording-flow checks; no board access."""
import hashlib
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch, MagicMock

import record_tilts as tilts


class TiltTests(unittest.TestCase):
    def test_reference_signs_and_gravity_length(self):
        h = 9.80665 / math.sqrt(2)
        cases = [((0, 0), [0, 0, 9.80665]), ((45, 0), [0, h, h]),
                 ((-45, 0), [0, -h, h]), ((0, 45), [-h, 0, h]), ((0, -45), [h, 0, h])]
        for angles, expected in cases:
            for actual, wanted in zip(tilts.reference_vector(*angles), expected):
                self.assertAlmostEqual(actual, wanted)
        vector = tilts.reference_vector(35, -20)
        self.assertAlmostEqual(sum(x*x for x in vector), 9.80665**2)

    def test_nominal_is_not_marked_as_measured(self):
        self.assertEqual(tilts.read_angles('', [45, 0]), ([45, 0], 'nominal_target_only'))
        self.assertEqual(tilts.read_angles('44.5,0.2', [45, 0])[1], 'operator_entered_reference')
        for text in ('nan,0', 'inf,0', '100,0', '45', '0,0,0'):
            with self.assertRaises(ValueError):
                tilts.read_angles(text, [45, 0])

    def test_fixed_candidate_matches_known_reference(self):
        reference = tilts.reference_vector(45, 0)
        candidate = {'bias_sensor_m_s2': [1, 2, 3], 'correction_sensor_to_body': [[1,0,0],[0,1,0],[0,0,1]]}
        mean = dict(zip(('ax','ay','az'), [reference[i]+candidate['bias_sensor_m_s2'][i] for i in range(3)]))
        result = tilts.compare(mean, candidate, [45, 0])
        self.assertAlmostEqual(result['direction_difference_deg'], 0, places=5)
        self.assertAlmostEqual(result['vector_difference_m_s2'], 0)

    def test_four_runs_and_resume_do_not_repeat_completed_poses(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root/'firmware').mkdir()
            (root/'firmware/main.cpp').write_text('synthetic')
            digest = hashlib.sha256(b'synthetic').hexdigest()
            baseline = root/'baseline'
            baseline.mkdir()
            (baseline/'pose/attempt').mkdir(parents=True)
            (baseline/'pose/attempt/summary.json').write_text(json.dumps({'serial_sha256':'test-log'}))
            (baseline/'session.json').write_text(json.dumps({'source_sha256':digest,'completed':['pose'],'reference_configuration':['synthetic']}))
            candidate = root/'candidate.json'
            candidate.write_text(json.dumps({'source_sha256':digest, 'bias_sensor_m_s2':[0,0,0],
                'correction_sensor_to_body':[[1,0,0],[0,1,0],[0,0,1]], 'records':[{'pose':'pose','original_sha256':'test-log'}]}))
            args = ['--port','FAKE','--baseline-session',str(baseline),'--calibration',str(candidate)]
            result = {'mean_physical':{'ax':0,'ay':0,'az':9.80665}, 'samples':20000, 'observed_average_hz':1000}
            with patch.object(tilts,'ROOT',root), patch.object(tilts.serial,'Serial',return_value=MagicMock()), patch.object(tilts.recorder,'capture',return_value=result) as capture, patch('builtins.input',side_effect=['jig','1','']+['']*4), patch('builtins.print'):
                tilts.main(args)
                self.assertEqual(capture.call_count,4)
                folder = next((root/'artifacts/tilt-recordings').iterdir())
                self.assertEqual(len(json.loads((folder/'session.json').read_text())['completed']),4)
                capture.reset_mock()
                with patch('builtins.input',return_value=''):
                    tilts.main(args+['--resume',str(folder)])
                capture.assert_not_called()


if __name__ == '__main__':
    unittest.main(verbosity=2)
