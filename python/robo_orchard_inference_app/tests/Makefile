# Put it first so that "make" without argument is like "make help".
PYTEST_OPT := -rf -x -n 2 --dist loadgroup

test:
	mkdir -p ../build/test && cd ../build/test && pytest -x -c ../../tests/pytest.ini $(PYTEST_OPT) ../../tests/python --alluredir allure_unittest/ \
	--cov-config=../../tests/.coveragerc --cov --cov-report=term --cov-report=xml  \
	--cov-report=html --junitxml=unittest.xml --html=unittest.html

.PHONY: test
