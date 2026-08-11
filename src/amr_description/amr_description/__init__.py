"""Shared fleet configuration for the AMR fleet.

Ships next to config/fleet.yaml so that the typed robot list and the code that
interprets it stay together. Every launch file in the workspace consumes the fleet
through this module rather than parsing the YAML itself.
"""
