from senior_control.safety.guard import command_expired


def test_command_older_than_ttl_expires_without_stop_message():
    assert command_expired(10.0, 10.5, 0.5) is False
    assert command_expired(10.0, 10.5 + 1e-9, 0.5) is True
    assert command_expired(None, 10.0, 0.5) is True
    assert command_expired(10.0, 10.0, None) is True
    assert command_expired(10.0, 12.0, 0.0) is True
