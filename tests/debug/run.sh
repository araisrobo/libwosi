# must 'make install' before doing this
gcc -g -O0 -I/usr/local/include/modbus -lmodbus mou-unit-test-slave.c -o slave
gcc -g -O0 -I../../src -lmodbus mou-unit-test-master.c -o master
