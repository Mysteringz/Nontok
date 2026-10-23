from lab_assistant.voice.commands import Command, Intent, grammar, parse_command


def test_find():
    assert parse_command("find the multimeter") == Command(Intent.FIND, item="multimeter")
    assert parse_command("bring me the screw driver") == Command(Intent.FIND, item="screwdriver")


def test_find_unknown_item_is_ignored():
    assert parse_command("find the oscilloscope") is None


def test_take_to():
    assert parse_command("take this to storage") == Command(Intent.TAKE_TO, place="storage")
    assert parse_command("take it to the work station") == Command(Intent.TAKE_TO, place="workstation")


def test_go_back_and_stop():
    assert parse_command("go back") == Command(Intent.GO_BACK)
    assert parse_command("please stop") == Command(Intent.STOP)


def test_noise():
    assert parse_command("") is None
    assert parse_command("[unk]") is None
    assert parse_command("hello there") is None


def test_every_grammar_phrase_parses():
    for phrase in grammar():
        if phrase != "[unk]":
            assert parse_command(phrase) is not None, phrase
