# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'XRSLAM'
copyright = '2023, XRSLAM Authors'
author = 'XRSLAM Authors'

import os
import re
import shutil
import subprocess
import sys
import sphinx_rtd_theme
def build_doxygen_docs(temp_dir='doxygen', cpp_dir='cpp_api'):
    """Build sphinx docs for C++"""
    cmd = ['doxygen', 'Doxyfile.in']
    subprocess.check_call(
        cmd,
        stdout=sys.stdout,
        stderr=sys.stderr)
    # move generated results to _build
    doxygen_dir = os.path.join(temp_dir, 'html')
    dst_dir = os.path.join('_build', 'html', cpp_dir)
    if os.path.exists(dst_dir):
        shutil.rmtree(dst_dir)
    shutil.copytree(doxygen_dir, dst_dir)
    if os.path.exists(temp_dir):
        shutil.rmtree(temp_dir)
        
# build c++ doxygen
build_doxygen_docs()

# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    'sphinx.ext.autodoc', 'sphinx.ext.napoleon', 'sphinx.ext.viewcode',
    'sphinx_markdown_tables', 'sphinx_copybutton', 'myst_parser'
]

autodoc_mock_imports = ['xrslam_cpp']

# Parse `Returns` in docstr with parameter style
napoleon_custom_sections = [('Returns', 'params_style')]

# Ignore >>> when copying code
copybutton_prompt_text = r'>>> |\.\.\. '
copybutton_prompt_is_regexp = True

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# The suffix(es) of source filenames.
# You can specify multiple suffix as a list of string:
#
source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'sphinx_rtd_theme'
html_theme_path = [sphinx_rtd_theme.get_html_theme_path()]

# Enable ::: for my_st
myst_enable_extensions = ['colon_fence']

master_doc = 'index'
