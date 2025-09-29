# File Organization

All example files can be kept in "ESP-Examples".

- To work through a new example, create a new .c file in the ESP-Examples directory
- To switch the app_main() that runs upon building and flashing,
  change the "SRCS" in "/ESP-Projects/ESP-Examples/CMakeLists.txt" to include the file with your main func and any possible header files you create
