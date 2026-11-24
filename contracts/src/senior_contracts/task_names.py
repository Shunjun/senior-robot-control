"""Task display-name normalization shared by the domain and wire boundary."""

# Explicit union of Python/JavaScript boundary whitespace; preserve interior characters.
TASK_NAME_WHITESPACE = (
    "\u0009\u000a\u000b\u000c\u000d"
    "\u001c\u001d\u001e\u001f\u0020\u0085\u00a0\u1680"
    "\u2000\u2001\u2002\u2003\u2004\u2005\u2006\u2007\u2008\u2009\u200a"
    "\u2028\u2029\u202f\u205f\u3000\ufeff"
)


def normalize_task_name(value: object) -> str:
    """Trim the agreed boundary set and require 1–128 Unicode code points."""
    if not isinstance(value, str):
        raise ValueError("name must be text")
    name = value.strip(TASK_NAME_WHITESPACE)
    if not 1 <= len(name) <= 128:
        raise ValueError("name must contain 1–128 Unicode characters")
    return name
