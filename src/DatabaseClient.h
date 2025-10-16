#include <string>
#include <vector>


#ifndef SMARTTRASHCAN_DATABASECLIENT_H
#define SMARTTRASHCAN_DATABASECLIENT_H


#define ENABLE_USER_AUTH
#define ENABLE_DATABASE


#include <FirebaseClient.h>

using AsyncClient = AsyncClientClass;


class DatabaseClient {

	public:
		static DatabaseClient& getInstance();

		void setup();
		void loop();
		RealtimeDatabase& getRealtimeDatabase();
		AsyncClient& getAsyncClient();

		void handleSerialCommands(std::vector<std::string> commands);

	private:
		DatabaseClient();
		DatabaseClient(const DatabaseClient&) = delete;
		DatabaseClient& operator=(const DatabaseClient&) = delete;
};

extern DatabaseClient& databaseClient;

#endif
