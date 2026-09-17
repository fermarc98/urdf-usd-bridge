# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Guarded access to the ``pxr`` modules.

We code against the OpenUSD Python API only. We deliberately do not declare a
hard dependency on any particular distribution of it, because the two that
matter are packaged very differently:

* ``usd-core`` -- Pixar's OpenUSD build, published for Linux, Windows and macOS.
* ``usd-exchange`` -- NVIDIA's OpenUSD Exchange SDK, which vendors ``pxr`` *and*
  adds ``usdex.core``. **No macOS wheels are published**, so anything that needs
  it is Linux/Windows only.

Installing both into one environment gives two copies of the USD libraries and
is not supported.
"""

from __future__ import annotations

_INSTALL_HINT = (
    "No OpenUSD Python bindings found (`pxr` is not importable).\n"
    "Install exactly one of:\n"
    "    pip install 'urdf-usd-bridge[core]'      # usd-core, all platforms\n"
    "    pip install 'urdf-usd-bridge[exchange]'  # usd-exchange, Linux/Windows only"
)


class UsdUnavailableError(RuntimeError):
    """Raised when the ``pxr`` modules cannot be imported."""


def require_pxr() -> None:
    """Raise :class:`UsdUnavailableError` with an install hint if ``pxr`` is missing.

    Modules that need OpenUSD import it at module scope in the ordinary way;
    this exists so the CLI can turn the resulting ``ImportError`` into advice
    the user can act on.
    """
    try:
        import pxr  # noqa: F401
    except ImportError as exc:  # pragma: no cover - environment dependent
        raise UsdUnavailableError(_INSTALL_HINT) from exc


def pxr_available() -> bool:
    """Whether the ``pxr`` modules can be imported."""
    try:
        require_pxr()
    except UsdUnavailableError:
        return False
    return True


def usd_version() -> str | None:
    """The OpenUSD version string, or ``None`` when unavailable."""
    try:
        from pxr import Usd
    except ImportError:  # pragma: no cover - environment dependent
        return None
    return ".".join(str(part) for part in Usd.GetVersion())


def try_register_newton_schemas() -> bool:
    """Best-effort registration of the codeless Newton USD schemas.

    ``newton-usd-schemas`` is a pure-python wheel, so this works on every
    platform. It is optional: inspection reads authored attribute names
    directly and never depends on schema registration.

    Returns:
        True when the ``newton`` plugin is registered after this call.
    """
    try:
        from pxr import Plug
    except ImportError:  # pragma: no cover - environment dependent
        return False
    registry = Plug.Registry()
    if registry.GetPluginWithName("newton"):
        return True
    try:
        import newton_usd_schemas  # noqa: F401
    except ImportError:
        return False
    return bool(registry.GetPluginWithName("newton"))
