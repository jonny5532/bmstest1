cd $(dirname "$0")
docker run -i --rm -v $PWD/..:/app -w /app $(docker build -q .) bash -c "cd tests/build; make || exit; ./dump_current_limits > current_limits; ./test_contactors; ./test_contactors_rev1; ./test_low_voltage; ./test_soc; ./test_events; ./test_balancing; ./test_current_limits; ./test_sampler; ./test_system_sm;"

