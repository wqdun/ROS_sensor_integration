// 3 6
// 10 15 12
// 1 9 12 23 26 37

#include <iostream>
#include <vector>
#include <map>

void GetBoundary(const std::vector<int> &numOfRooms, std::vector<int> &boundaries) {
    int sum = 0;
    for (int i = 0; i < numOfRooms.size(); ++i) {
        sum += numOfRooms[i];
        boundaries.push_back(sum);
    }
}

struct BuildingIdAndRoomId {
    int buildingId_;
    int roomId_;

    BuildingIdAndRoomId(): buildingId_(-1), roomId_(-1) {
    }
};

int main() {
    const std::vector<int> numOfRooms {10, 15, 12};
    const std::vector<int> roomId {7, 0, 1, 9, -12, 23, 26, 88, 45};

    BuildingIdAndRoomId buildingIdAndRoomId;
    std::map<int, BuildingIdAndRoomId> buildingIdAndRoomIdMap;
    for (auto i: roomId) {
        buildingIdAndRoomIdMap.insert({i, buildingIdAndRoomId});
    }

    std::vector<int> boundaries {0};
    GetBoundary(numOfRooms, boundaries);

    auto mapIter = buildingIdAndRoomIdMap.begin();
    for (int buildingId = 1; buildingId != boundaries.size(); ++buildingId) {
        for ( ; mapIter != buildingIdAndRoomIdMap.end(); ++mapIter) {
            if (mapIter->first <= 0) {
                continue;
            }
            if (mapIter->first > boundaries[buildingId]) {
                break;
            }

            // std::cout << mapIter->first << " is in " << buildingId << " building, " << mapIter->first - boundaries[buildingId - 1] << " room.\n";
            mapIter->second.buildingId_ = buildingId;
            mapIter->second.roomId_ = mapIter->first - boundaries[buildingId - 1];
        }
    }

    for (auto i: buildingIdAndRoomIdMap) {
        if (i.second.buildingId_ < 0) {
            std::cout << "Illegal room " << i.first << "\n";
            continue;
        }
        std::cout << i.first << " is in " << i.second.buildingId_ << " building, " << i.second.roomId_ << " room.\n";
    }
}

