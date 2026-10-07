"""FastAPI service for isolated offline experiments."""

def create_app(*args, **kwargs):
    # SSH transport and the standalone remote helper do not require FastAPI.
    from .app import create_app as factory
    return factory(*args, **kwargs)

__all__ = ["create_app"]
