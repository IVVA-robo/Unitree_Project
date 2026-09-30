"""Offline tests: no robot, network, ROS nodes, service calls or live configuration."""
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch
import tarfile

import audit
import collect_robot
import restore_model
import snapshot


class ExportTests(unittest.TestCase):
    def test_forbidden_paths(self):
        for name in ('../bad', '/absolute', 'a/.ssh/id_ed25519', 'key.pem',
                     'a/.env', 'a/site.local.env', 'a/site.venue.env', 'a/file.partial'):
            self.assertTrue(snapshot.forbidden(name), name)
        self.assertFalse(snapshot.forbidden('config/robot_pov.r1.env'))

    def test_sanitization(self):
        secret = b'ghp_' + b'A' * 36
        self.assertIsNone(snapshot.sanitize(secret))
        self.assertNotIn(b'A' * 32, snapshot.sanitize(b'commissioning_token="' + b'A' * 32 + b'"'))
        self.assertEqual(snapshot.sanitize(b'ordinary source'), b'ordinary source')

    def test_repack_without_extracting(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            old, new = root / 'old.tar', root / 'new.tar.gz'
            with tarfile.open(old, 'w') as archive:
                for name in ('src/a.py', '../escape', '.ssh/id_rsa', 'build/output', '.cache/model'):
                    item = tarfile.TarInfo(name)
                    item.size = 4
                    archive.addfile(item, io.BytesIO(b'test'))
            with patch.object(snapshot, 'ROOT', root):
                snapshot.repack(old, new, [])
            with tarfile.open(new) as archive:
                self.assertEqual(archive.getnames(), ['src/a.py'])

    def model_fixture(self, directory):
        data = b'GGUF-small-test'
        digest = hashlib.sha256(data).hexdigest()
        (directory / 'part-0000').write_bytes(data)
        manifest = {'parts': [{'file': 'part-0000', 'bytes': len(data), 'sha256': digest}],
                    'bytes': len(data), 'sha256': digest}
        (directory / 'manifest.json').write_text(json.dumps(manifest))
        return data, manifest

    def test_model_roundtrip_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            data, _ = self.model_fixture(root)
            output = root / 'model.gguf'
            restore_model.restore(root, output)
            self.assertEqual(output.read_bytes(), data)
            with self.assertRaises(FileExistsError):
                restore_model.restore(root, output)

    def test_model_corruption_and_path_traversal(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _, manifest = self.model_fixture(root)
            (root / 'part-0000').write_bytes(b'wrong')
            with self.assertRaises(ValueError):
                restore_model.restore(root, root / 'bad-output')
            manifest['parts'][0]['file'] = '../escape'
            (root / 'manifest.json').write_text(json.dumps(manifest))
            with self.assertRaises(ValueError):
                restore_model.restore(root, root / 'not-created')
            self.assertFalse((root / 'not-created').exists())

    def test_collector_only_readonly_ssh_and_receipt(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / 'export'
            calls = []
            def run(argv, **kwargs):
                calls.append((argv, kwargs))
                code = kwargs['input']
                compile(code, '<remote-collector>', 'exec')
                if 'capture_output' in kwargs:
                    return subprocess.CompletedProcess(argv, 0, json.dumps({'hostname': 'fake', 'paths': [
                        {'path': '/home/unitree/VoiceAssistant', 'exists': True, 'readable': True}]}))
                kwargs['stdout'].write(b'fake transfer; no execution')
                return subprocess.CompletedProcess(argv, 0)
            with patch('sys.argv', ['collect_robot', '--output', str(target)]), patch.object(collect_robot.subprocess, 'run', run):
                self.assertEqual(collect_robot.main(), 0)
            self.assertEqual(len(calls), 2)
            self.assertTrue(all('StrictHostKeyChecking=yes' in call[0] for call in calls))
            self.assertEqual(target.stat().st_mode & 0o777, 0o700)
            self.assertFalse(json.loads((target / 'COMPLETE.json').read_text())['publication_review_complete'])

    def test_audit_markers_vs_credentials(self):
        audit.FINDINGS.clear()
        header = b'-----BEGIN ' + b'PRIVATE KEY-----'
        audit.scan_data(header, 'library-format-marker')
        self.assertEqual(audit.FINDINGS, [])
        audit.scan_data(header + b'\n' + b'A' * 64, 'test-key')
        self.assertEqual(audit.FINDINGS[0]['category'], 'private-key')
        audit.FINDINGS.clear()


if __name__ == '__main__':
    unittest.main()
