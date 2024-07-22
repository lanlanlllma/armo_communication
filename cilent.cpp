#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>

int main() {
    // Create a socket
    int clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket == -1) {
        std::cerr << "Failed to create socket." << std::endl;
        return 1;
    }

    // Set up the server address and port
    sockaddr_in serverAddress{};
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(8080);  // Replace with the actual server port
    if (inet_pton(AF_INET, "127.0.0.1", &(serverAddress.sin_addr)) <= 0) {
        std::cerr << "Invalid address/Address not supported." << std::endl;
        return 1;
    }

    // Connect to the server
    if (connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) < 0) {
        std::cerr << "Connection failed." << std::endl;
        return 1;
    }

    // Send and receive messages in a loop
    while (true) {
        std::string sendMessage;
        std::cout << "Enter message to send: ";
        std::getline(std::cin, sendMessage);

        if (send(clientSocket, sendMessage.c_str(), sendMessage.length(), 0) < 0) {
            std::cerr << "Failed to send message." << std::endl;
            break;
        }

        // Receive a response from the server
        char buffer[1024] = {0};
        if (recv(clientSocket, buffer, sizeof(buffer), 0) < 0) {
            std::cerr << "Failed to receive response." << std::endl;
            break;
        }

        std::cout << "Server response: " << buffer << std::endl;

        // Optionally, add an exit condition to break the loop
        if (sendMessage == "exit") {
            break;
        }

        // Sleep for a while before sending the next message
        sleep(1);
    }

    // Close the socket
    close(clientSocket);

    return 0;
}
