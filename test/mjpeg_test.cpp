// g++ -std=c++17 -Wall -Wextra -Werror -I components test/mjpeg_test.cpp -o /tmp/mjpeg && /tmp/mjpeg
#include <cassert>
#include <cstdint>
#include <cstring>
#include <vector>

#include "esp_video_camera/mjpeg.h"

using esphome::esp_video_camera::FrameHandoff;
using esphome::esp_video_camera::JpegRequestState;
using esphome::esp_video_camera::MJPEG_PART_HEADROOM;
using esphome::esp_video_camera::MJPEG_PART_TAILROOM;
using esphome::esp_video_camera::StopEpochLatch;
using esphome::esp_video_camera::StreamLifecycleAction;
using esphome::esp_video_camera::format_mjpeg_part;
using esphome::esp_video_camera::frame_mjpeg_in_place;
using esphome::esp_video_camera::mjpeg_half_resolution;

int main() {
  // 1. Multipart part header formatting and truncation detection.
  char part[96];
  int len = format_mjpeg_part(part, sizeof(part), 12345);
  const char expected[] =
      "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: 12345\r\n\r\n";
  assert(len == static_cast<int>(std::strlen(expected)));
  assert(std::strcmp(part, expected) == 0);
  char short_part[8];
  assert(format_mjpeg_part(short_part, sizeof(short_part), 12345) >=
         static_cast<int>(sizeof(short_part)));

  // 2. Half-resolution downscaling for landscape, portrait, and odd dimensions.
  uint16_t sw = 0, sh = 0;
  mjpeg_half_resolution(1280, 720, sw, sh);
  assert(sw == 640 && sh == 360);
  mjpeg_half_resolution(720, 1280, sw, sh);
  assert(sw == 360 && sh == 640);
  mjpeg_half_resolution(642, 362, sw, sh);
  assert(sw == 320 && sh == 180);

  // 3. In-place contiguous MJPEG framing (part header + JPEG + CRLF).
  const uint8_t fake_jpeg[] = {0xFF, 0xD8, 0x11, 0x22, 0x33, 0xFF, 0xD9};
  std::vector<uint8_t> raw(MJPEG_PART_HEADROOM + sizeof(fake_jpeg) + MJPEG_PART_TAILROOM, 0xAA);
  uint8_t *jpeg_buf = raw.data() + MJPEG_PART_HEADROOM;
  std::memcpy(jpeg_buf, fake_jpeg, sizeof(fake_jpeg));

  const uint8_t *framed = nullptr;
  size_t framed_len = 0;
  assert(frame_mjpeg_in_place(jpeg_buf, MJPEG_PART_HEADROOM, sizeof(fake_jpeg),
                              MJPEG_PART_TAILROOM, framed, framed_len));
  const char expected_hdr[] =
      "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: 7\r\n\r\n";
  const size_t hdr_len = std::strlen(expected_hdr);
  assert(framed == jpeg_buf - hdr_len);
  assert(framed_len == hdr_len + sizeof(fake_jpeg) + 2);
  assert(std::memcmp(framed, expected_hdr, hdr_len) == 0);
  assert(std::memcmp(framed + hdr_len, fake_jpeg, sizeof(fake_jpeg)) == 0);
  assert(framed[framed_len - 2] == '\r' && framed[framed_len - 1] == '\n');

  // Invalid framing cases fail closed.
  assert(!frame_mjpeg_in_place(jpeg_buf, 16, sizeof(fake_jpeg), MJPEG_PART_TAILROOM, framed, framed_len));
  assert(!frame_mjpeg_in_place(jpeg_buf, MJPEG_PART_HEADROOM, 0, MJPEG_PART_TAILROOM, framed, framed_len));
  assert(!frame_mjpeg_in_place(jpeg_buf, MJPEG_PART_HEADROOM, sizeof(fake_jpeg), 1, framed, framed_len));

  // 4. JpegRequestState handshake: happy path, stale wakeup, privacy-off abort,
  //    timeout/late-producer A->B race, and encode failure (0 bytes).
  JpegRequestState st{};
  // Producer cannot claim or complete when nothing is pending.
  assert(st.claim_request() == 0);
  assert(!st.complete_request(0, 1000));
  assert(!st.complete_request(1, 1000));

  // Happy path.
  uint32_t tok1 = st.begin_request();
  assert(st.pending);
  uint32_t claim1 = st.claim_request();
  assert(claim1 == tok1);
  assert(st.complete_request(claim1, 24500));
  assert(!st.pending);
  size_t out_len = 0;
  assert(st.finish_wait(tok1, true, true, out_len));
  assert(out_len == 24500);

  // Privacy switch OFF wakes waiter before producer encodes -> rejected even if
  // old jpeg_len is non-zero (generation mismatch + camera_streaming false).
  uint32_t tok2 = st.begin_request();
  assert(!st.finish_wait(tok2, true, false, out_len));
  assert(out_len == 0 && !st.pending);

  // Privacy OFF -> rapid ON wakeup without an encode -> generation mismatch still rejects!
  uint32_t tok3 = st.begin_request();
  assert(!st.finish_wait(tok3, true, true, out_len));
  assert(out_len == 0 && !st.pending);

  // Consumer timeout clears pending so a late producer aborts cleanly.
  uint32_t tok4 = st.begin_request();
  uint32_t claim4 = st.claim_request();
  assert(claim4 == tok4);
  assert(!st.finish_wait(tok4, false, true, out_len));
  assert(out_len == 0 && !st.pending);
  assert(!st.complete_request(claim4, 12345));

  // Late producer A -> B race: request A begins -> producer claims A -> A times out
  // -> request B begins -> producer completes A -> B must NOT be completed with A's
  // bytes and must remain pending until producer claims & completes B!
  uint32_t tokA = st.begin_request();
  uint32_t claimA = st.claim_request();
  assert(claimA == tokA);
  assert(!st.finish_wait(tokA, false, true, out_len));
  assert(out_len == 0 && !st.pending);

  uint32_t tokB = st.begin_request();
  assert(tokB != tokA && st.pending);
  assert(!st.complete_request(claimA, 11111));
  assert(st.pending);
  assert(st.done_gen != tokB);

  uint32_t claimB = st.claim_request();
  assert(claimB == tokB);
  assert(st.complete_request(claimB, 22222));
  assert(!st.pending);
  assert(st.finish_wait(tokB, true, true, out_len));
  assert(out_len == 22222);

  // Encode error (len == 0) wakes waiter and fails cleanly.
  uint32_t tok5 = st.begin_request();
  uint32_t claim5 = st.claim_request();
  assert(st.complete_request(claim5, 0));
  assert(!st.finish_wait(tok5, true, true, out_len));
  assert(out_len == 0);

  // 5. StopEpochLatch: rapid privacy OFF -> ON before capture_task_() observes OFF
  //    must still force a full STOP and invalidate pre-stop HTTP viewer sessions.
  StopEpochLatch latch{};
  bool camera_on = false;
  uint32_t stop_epoch = 0;

  // Initial idle -> start -> run.
  assert(latch.step(stop_epoch, false, camera_on) == StreamLifecycleAction::IDLE);
  assert(latch.step(stop_epoch, true, camera_on) == StreamLifecycleAction::START);
  camera_on = true;
  assert(latch.step(stop_epoch, true, camera_on) == StreamLifecycleAction::RUN);
  const uint32_t viewer_epoch = stop_epoch;
  assert(StopEpochLatch::session_valid(viewer_epoch, stop_epoch, true));

  // Rapid OFF -> ON toggle while camera_on is true: stop_epoch bumps, stream_requested
  // is already back to true before capture_task_() wakes up.
  ++stop_epoch;
  assert(!StopEpochLatch::session_valid(viewer_epoch, stop_epoch, true));
  assert(latch.step(stop_epoch, true, camera_on) == StreamLifecycleAction::STOP);

  // If VIDIOC_STREAMOFF fails, camera_on remains true and handled_epoch does NOT advance,
  // so the next loop iteration retries STOP instead of running or double-queuing.
  assert(latch.step(stop_epoch, true, camera_on) == StreamLifecycleAction::STOP);

  // Once VIDIOC_STREAMOFF succeeds, camera_on becomes false and handled_epoch advances;
  // next step cleanly starts a fresh stream.
  camera_on = false;
  latch.mark_stopped(stop_epoch);
  assert(latch.step(stop_epoch, true, camera_on) == StreamLifecycleAction::START);
  camera_on = true;
  assert(latch.step(stop_epoch, true, camera_on) == StreamLifecycleAction::RUN);

  // New viewer opened after restart has a new epoch and is valid.
  const uint32_t viewer2_epoch = stop_epoch;
  assert(StopEpochLatch::session_valid(viewer2_epoch, stop_epoch, true));

  // Privacy OFF while camera_on is false simply advances handled_epoch and stays IDLE.
  camera_on = false;
  ++stop_epoch;
  assert(latch.step(stop_epoch, false, camera_on) == StreamLifecycleAction::IDLE);
  assert(latch.handled_epoch == stop_epoch);
  assert(!StopEpochLatch::session_valid(viewer2_epoch, stop_epoch, true));

  // 6. FrameHandoff: every interleaving of normal handoff, timeout, stop, and
  //    rapid OFF/ON epoch transition never strands IN_USE or leaks pre-stop frames.
  FrameHandoff hf{};
  assert(hf.state() == FrameHandoff::IDLE);

  // 6a. Normal handoff: IDLE -> (1, REQUESTED) -> (1, IN_USE) -> IDLE.
  assert(hf.request(1));
  assert(hf.is_requested() && hf.is_requested_for(1) && !hf.is_requested_for(2) && !hf.is_in_use());
  assert(!hf.request(1));  // cannot double-request
  assert(!hf.deliver(2));  // wrong capture epoch cannot deliver
  assert(hf.deliver(1));
  assert(hf.is_in_use() && hf.epoch() == 1 && !hf.is_requested());
  assert(!hf.deliver(1));  // cannot double-deliver
  assert(hf.accept(1));
  assert(hf.is_in_use());
  hf.release();
  assert(hf.state() == FrameHandoff::IDLE);

  // 6b. Timeout BEFORE deliver: consumer cancels (2, REQUESTED) -> IDLE; late deliver fails.
  assert(hf.request(2));
  hf.cancel_or_release(2);
  assert(hf.state() == FrameHandoff::IDLE);
  assert(!hf.deliver(2));
  assert(hf.state() == FrameHandoff::IDLE);

  // 6c. Timeout RIGHT AFTER deliver: producer moved (3, REQUESTED) -> (3, IN_USE),
  //     consumer timed out and calls cancel_or_release(3) -> transitions IN_USE -> IDLE!
  assert(hf.request(3));
  assert(hf.deliver(3));
  assert(hf.is_in_use());
  hf.cancel_or_release(3);
  assert(hf.state() == FrameHandoff::IDLE);

  // 6d. Stop BEFORE deliver: wake_all_waiters_ cancels REQUESTED -> IDLE;
  //     producer deliver fails; consumer accept(4, false) leaves IDLE.
  assert(hf.request(4));
  hf.cancel_request();
  assert(hf.state() == FrameHandoff::IDLE);
  assert(!hf.deliver(4));
  assert(!hf.accept(4, false));
  assert(hf.state() == FrameHandoff::IDLE);

  // 6e. Stop AFTER deliver before consumer checks stream_valid:
  //     cancel_request() preserves IN_USE (so active inference isn't clobbered),
  //     then consumer accept(5, false) immediately releases IN_USE -> IDLE!
  assert(hf.request(5));
  assert(hf.deliver(5));
  hf.cancel_request();
  assert(hf.is_in_use());
  assert(!hf.accept(5, false));
  assert(hf.state() == FrameHandoff::IDLE);

  // 6f. Stop DURING active inference: cancel_request() preserves IN_USE until
  //     detector finishes and calls release().
  assert(hf.request(6));
  assert(hf.deliver(6));
  assert(hf.accept(6, true));
  hf.cancel_request();
  assert(hf.is_in_use());
  hf.release();
  assert(hf.state() == FrameHandoff::IDLE);

  // 6g. Rapid OFF/ON epoch race: producer captures at epoch N while consumer A
  //     requested at epoch N. Privacy OFF cancels (N, REQUESTED) -> IDLE, then
  //     privacy ON admits consumer B at epoch N+1. Producer's late deliver(N)
  //     MUST fail and MUST NOT deliver the pre-stop frame to consumer B; stale
  //     consumer A's cancel_or_release(N) MUST NOT disturb consumer B either.
  constexpr uint32_t epoch_n = 10;
  constexpr uint32_t epoch_next = epoch_n + 1;
  assert(hf.request(epoch_n));
  hf.cancel_request();  // privacy OFF
  assert(hf.state() == FrameHandoff::IDLE);
  assert(hf.request(epoch_next));  // privacy ON + new consumer B
  assert(!hf.deliver(epoch_n));    // pre-stop producer at epoch N is rejected!
  assert(hf.state() == FrameHandoff::REQUESTED && hf.epoch() == epoch_next);
  hf.cancel_or_release(epoch_n);   // late abort from consumer A cannot touch N+1
  assert(hf.state() == FrameHandoff::REQUESTED && hf.epoch() == epoch_next);
  assert(!hf.accept(epoch_n, true));
  assert(hf.state() == FrameHandoff::REQUESTED && hf.epoch() == epoch_next);
  // Post-restart producer at epoch N+1 delivers cleanly to consumer B.
  assert(hf.deliver(epoch_next));
  assert(hf.accept(epoch_next));
  assert(hf.is_in_use() && hf.epoch() == epoch_next);
  hf.release();
  assert(hf.state() == FrameHandoff::IDLE);

  return 0;
}
