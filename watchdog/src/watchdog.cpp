#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/watchdog.h>
#include <cstring>
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    const char *wdog_device = "/dev/watchdog";
    int fd = open(wdog_device, O_RDWR);
    if (fd < 0) {
        std::cerr << "Failed to open " << wdog_device
                  << ": " << std::strerror(errno) << "\n";
        return 1;
    }

    // Set the timeout (in seconds)
    int timeout = 25;  // e.g. 10 seconds
    if (ioctl(fd, WDIOC_SETTIMEOUT, &timeout) < 0) {
        std::cerr << "Failed to set watchdog timeout: "
                  << std::strerror(errno) << "\n";
        close(fd);
        return 1;
    }
    std::cout << "Watchdog timeout set to " << timeout << " seconds.\n";

    // Read back to confirm
    if (ioctl(fd, WDIOC_GETTIMEOUT, &timeout) < 0) {
        std::cerr << "Failed to get watchdog timeout: "
                  << std::strerror(errno) << "\n";
        close(fd);
        return 1;
    }
    std::cout << "Confirmed watchdog timeout is " << timeout << " seconds.\n";

    // Main loop: pet (keepalive) the watchdog every few seconds
    while (true) {
        // Writing any data to /dev/watchdog "pets" it
        if (write(fd, "\0", 1) != 1) {
            std::cerr << "Watchdog keepalive failed: "
                      << std::strerror(errno) << "\n";
            break;
        }
        std::cout << "Watchdog pinged.\n";

        // Sleep for less than the timeout
        std::this_thread::sleep_for(std::chrono::seconds(timeout / 2));
    }

    // Disable the watchdog before exiting (optional, may trigger reboot)
    int disable = WDIOS_DISABLECARD;
    if (ioctl(fd, WDIOC_SETOPTIONS, &disable) < 0) {
        std::cerr << "Failed to disable watchdog: "
                  << std::strerror(errno) << "\n";
    }

    close(fd);
    return 0;
}
