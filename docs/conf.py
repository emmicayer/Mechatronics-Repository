import os
import sys
# sys.path.insert(0, os.path.abspath(".."))

# Add the on_board folder so modules like boot, main, etc. can be imported
sys.path.insert(0, os.path.abspath("../Final Term Project/on_board"))




# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Mechatronics Term Project'
copyright = '2025, Emmi Cayer & Erin Maxwell'
author = 'Emmi Cayer & Erin Maxwell'
release = '1'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = ["sphinx.ext.autodoc", "sphinx.ext.napoleon"]

autodoc_default_options = {
    "members": True,
    "undoc-members": True,
    "special-members": "__init__",
}


templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

autodoc_mock_imports = ["pyb", "time"]


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

import sphinx_rtd_theme

html_theme = "sphinx_rtd_theme"
html_theme_path = [sphinx_rtd_theme.get_html_theme_path()]

html_static_path = ['_static']
