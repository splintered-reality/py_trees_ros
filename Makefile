docs:
	sphinx-build -v -b html doc html

clean:
	rm -rf build html

.PHONY: docs clean
