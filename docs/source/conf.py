# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'abv_gnc'
copyright = '2026, The Autonomy Lab'
author = 'Sam Lovelace'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "myst_parser",
]

# Auto-generates an #anchor for every heading (up to h3) so pages can
# cross-reference a specific section of another page, e.g.
# [AbvState](../interface/abv_msgs.md#abvstate).
myst_heading_anchors = 3

source_suffix = {
    ".rst": "restructuredtext",
    ".md": "markdown",
}


templates_path = ['_templates']
exclude_patterns = []

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_theme_options = {
    "navigation_depth": 4,
}

html_static_path = ['_static', '../media']
html_extra_path = ['../media']

# conf.py
import mimetypes
mimetypes.add_type("video/mp4", ".mp4")
