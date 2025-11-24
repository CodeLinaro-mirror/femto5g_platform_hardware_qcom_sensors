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

#include "SensorsEventCallback.h"

SensorsEventCallback::SensorsEventCallback(std::queue<sensors_event_t>& fifo, std::mutex& fifoLock,
                                           std::condition_variable& waitCV)
  : mFifo(fifo), mFifoLock(fifoLock), mWaitCV(waitCV) {}

void SensorsEventCallback::postEvents(const std::vector<sensors_event_t>& events) {
  std::lock_guard<std::mutex> lock(mFifoLock);
  for (size_t i = 0; i < events.size(); ++i) {
    if (mFifo.size() > kMaxFifoSize) {
      mFifo.pop();
    }
    mFifo.push(events[i]);
  }
  mWaitCV.notify_all();
};
