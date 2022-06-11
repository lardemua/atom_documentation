# ATOM Documentation

**[https://lardemua.github.io/atom_documentation/](https://lardemua.github.io/atom_documentation/)**

This repository contains the documentation for the [ATOM Calibration Framework](https://github.com/lardemua/atom) software.

We use [mkdocs](https://www.mkdocs.org/) to write the documentation in markdown files which are then compiled to html using mkdocs.

# Usage

To create or revise the documentation you should first launch a visualizer:

    mkdocs serve

and then launch a continuous compilation as you are working on the files:

    watch -n 1 mkdocs build

finally, start a browser and go to [http://127.0.0.1:8000/](http://127.0.0.1:8000/).


# Deployment

If you are happy with the changes you can deploy them to public site.

!!! Warning "Make sure you have committed and pushed all your changes to the main branch."

We are using the Project Pages, which are simpler as the site files get deployed to a branch within the project repository (gh-pages).
Just run:

    mkdocs gh-deploy

and your changes will be pushed to the gh-pages branch, and shortly after available at:

[https://lardemua.github.io/atom_documentation/](https://lardemua.github.io/atom_documentation/)


# Installation

Install mkdocs from [here](https://www.mkdocs.org/getting-started/#installation).