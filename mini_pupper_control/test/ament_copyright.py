from ament_copyright.main import main
import pytest


@pytest.mark.linter
def test_copyright():
    report = main(argv=['.'])
    assert report == 0, 'Found errors'
