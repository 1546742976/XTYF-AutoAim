"""FastAPI service for isolated offline experiments."""

from .app import create_app

__all__ = ["create_app"]
