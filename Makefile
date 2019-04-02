docs:
	sphinx-build -b html doc html

clean:
	rm -rf build html

.PHONY: docs clean
