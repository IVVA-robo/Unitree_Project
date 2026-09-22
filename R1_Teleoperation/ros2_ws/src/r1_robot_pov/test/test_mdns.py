from types import SimpleNamespace

import r1_robot_pov.cli as cli


def test_mdns_publishes_service_without_invalid_alias(monkeypatch):
    commands = []

    class FakeProcess:
        def __init__(self, command):
            self.command = command
            self.returncode = None

        def poll(self):
            return self.returncode

        def terminate(self):
            self.returncode = 0

        def wait(self, timeout=None):
            self.returncode = 0

        def kill(self):
            self.returncode = -9

    def fake_popen(command, **_kwargs):
        commands.append(command)
        return FakeProcess(command)

    monkeypatch.setattr(cli.shutil, 'which', lambda name: '/usr/bin/avahi-publish')
    monkeypatch.setattr(cli.subprocess, 'Popen', fake_popen)
    config = SimpleNamespace(mdns_name='IONOS-ROBOTS.local', lan_ip='192.168.8.9', port=8080)

    publisher = cli.MdnsPublisher(config)
    publisher.start()

    assert commands[0][0:5] == [
        '/usr/bin/avahi-publish', '-s', 'Robot POV', '_http._tcp', '8080'
    ]
    publisher.stop()
