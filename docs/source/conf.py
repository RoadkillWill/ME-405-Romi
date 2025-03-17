# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

import os
import sys
sys.path.insert(0, r"C:\Users\willh\.vscode\ME405\ME-405-Romi\code")

project = 'ME 405 Romi'
copyright = '2025, Will Hite and Ryan Maldonado'
author = 'Will Hite and Ryan Maldonado'
release = '1.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',  # For Google-style docstrings
    'sphinx.ext.viewcode'  # Adds links to source code
]

templates_path = ['_templates']
exclude_patterns = []



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

# Set HTML theme (optional)
html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
html_css_files = ['style.css']
html_use_index = True
html_domain_indices = True
