.PHONY: all
all: venv build

.PHONY: venv
venv:
	test -d venv || python3 -mvenv --system-site-packages venv

.PHONY: build
build:
	. venv/bin/activate && cd ahrs-py && maturin develop

.PHONY: install
install:
	. venv/bin/activate && python3 -m ipykernel install --user --name=ahrs

