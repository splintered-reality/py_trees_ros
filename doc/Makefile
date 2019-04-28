all:
	@echo "This makefile is for building documentation"
	@echo "in the virtual environment via '. ./venv.bash'"
	@echo ""
	@echo "Valid targets:"
	@echo ""
	@echo "    docs: build the sphinx documentation in ./html"
	@echo "    clean: clean out the html directory"

docs:
	sphinx-build -v -b html . html

clean:
	rm -rf html

.PHONY: docs clean
