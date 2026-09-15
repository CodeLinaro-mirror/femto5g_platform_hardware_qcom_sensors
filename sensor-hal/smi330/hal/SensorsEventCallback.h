/*
 * Copyright (C) 2025 Robert Bosch GmbH
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SENSORS_EVENT_CALLBACK_H_
#define SENSORS_EVENT_CALLBACK_H_

#include <hardware/sensors.h>

#include <condition_variable>
#include <mutex>
#include <queue>
#include <vector>

class SensorsEventCallback {
public:
  SensorsEventCallback(std::queue<sensors_event_t>& fifo, std::mutex& fifoLock, std::condition_variable& waitCV);
  void postEvents(const std::vector<sensors_event_t>& events);

private:
  static constexpr size_t kMaxFifoSize = 512;
  std::queue<sensors_event_t>& mFifo;
  std::mutex& mFifoLock;
  std::condition_variable& mWaitCV;
};

#endif  // SENSORS_EVENT_CALLBACK_H_
