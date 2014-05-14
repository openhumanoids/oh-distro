all : send_message listener

send_message : send_message.c exlcm_example_t.c
	gcc -o send_message send_message.c exlcm_example_t.c -llcm -L/usr/local/lib

listener : listener.c exlcm_example_t.c
	gcc -o listener listener.c exlcm_example_t.c -llcm -L/usr/local/lib


clean : 
	rm listener send_message