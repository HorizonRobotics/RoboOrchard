# Minimal makefile for Sphinx dumentation
#

# You can set these variables from the command line.
SPHINXOPTS    = "-W"
# SPHINXOPTS    =
SPHINXBUILD   = sphinx-build
SPHINXPROJ    = RoboOrchardInferenceApp
SOURCEDIR     = .
BUILDDIR      = ../build/docs_build
ROBO_ORCHARD_DOCS_CURRENT_VERSION_ID ?= master

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@mkdir -p $(BUILDDIR)
	$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
	@cp -rf $(BUILDDIR)/html $(BUILDDIR)/$(ROBO_ORCHARD_DOCS_CURRENT_VERSION_ID)

clean:
	@rm -rf $(BUILDDIR)

.PHONY: help Makefile
