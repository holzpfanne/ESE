# Queue task

3 tasks cout a global variable up and down and put that as string package into a Queue.

The monitoring task takes out a package and prints it to the the PC via UART.

## Setting timeout via Terminal

To be able to read non locking, evertime a character is received a interrupt is triggered to write the char into buffer.

Everytime the Monitoring task runs it check if an message is in the input buffer and proccesses it and puts it into the delay Queue

The Producer task get the delay data and checks if the delay producer is the producer running, if so the delay is set, if not the data is put into the queue again