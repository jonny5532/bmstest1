docker run -it --rm -v $PWD/..:/app -w /app $(docker build -q .) bash -c "cd tests/build; make; ./test_soc"

