# BMS Firmware Update over the HMI Serial Link: Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let the ESP32-P4 HMI flash a new controller image into the RP2354B over the existing HMI serial bus. That removes the need for a USB cable or the Raspberry Pi, and lets a battery's controller and HMI be updated together with one HMI OTA. Over serial, only signed images built for that exact pack are accepted.

**Architecture:**
- **Two slots on the RP2354B.** Its flash gets an RP2350 partition table with A/B image slots. The boot ROM picks the slot, and "try before you buy" (TBYB) rolls back automatically.
- **The running BMS firmware writes the update.** It receives the new image over the HMI serial protocol (new message types 0x10–0x14), writes it into the inactive slot and checks it. It then reboots into the new slot with a flash-update reboot.
- **The new image confirms itself.** It "buys" itself once it has initialised and the HMI link works. If it never does, the boot ROM goes back to the old slot.
- **The HMI carries the matching image.** Each HMI build includes the controller image for its battery (ck0: BLUETESLA/REV1, ck1: LFP1/REV0), so the screen's event table and the controller always match. The HMI sends the image only when the user asks, from the settings page.
- **Serial OTA images must be signed (in-firmware check, no OTP secure boot).**
  - The BMS checks an Ed25519 signature over the whole image before activating it.
  - It trusts our key (compiled in) plus up to 4 user keys that can **only** be added over USB.
  - USB / picotool stays fully open: physical access is trusted.
- **Each image carries a manifest** inside it, covered by the signature. A serial OTA is only accepted if the image's **chemistry, cell count and cell-presence mask** match the pack identity recorded when the battery was provisioned over USB.
- **Every image checks the real pack at boot,** however it was installed. If the cells the BMBs report don't match the image's cell count/mask (or the voltages are impossible for its chemistry), it refuses to close the contactors.

**No custom bootloader:** the RP2350 boot ROM already does partition selection, address translation, TBYB and rollback. Signature and identity checks run in the application, before it activates the new slot.

**Tech stack:** pico-sdk 2.2.0 (`pico_bootrom`, `pico_sha256`, `hardware_flash`, `pico_flash`), Monocypher (Ed25519 verify), picotool 2.1+, ESPHome external component `hmi_bms` (C++ / Python codegen).

---

## How hard is it?

**Moderate to substantial: about 2 weeks including bench testing.** No new hardware is needed: the existing TX/RX pair (Pico GPIO38/39, 460800 baud, `0xFF`/length/CRC16 framing) is enough. The hard parts are not the transfer itself:

1. **One-time migration to a partition table.** Every controller needs one last flash over USB: ck0 through the Pi with picotool, ck1 through its Pi. This moves the app from flash offset 0 into slot A.
2. **XIP address translation.** When the app runs from a partition, `XIP_BASE + offset` is translated into that partition. [nvm.c](../../../bms/drivers/chip/nvm.c) reads the LittleFS/NVM area at `XIP_BASE + 0x1E0000`. Once the app runs from a slot, that read lands somewhere else, so settings appear blank.
   - **Fix:** read through the untranslated XIP alias (`XIP_NOCACHE_NOALLOC_NOTRANSLATE_BASE`). Do this before the first partitioned build ships, or settings will look reset.
3. **Safety gating.** Flash erase and program run with interrupts off and core 1 parked (`flash_safe_execute`). For the length of each operation that stalls can2040 (PIO CAN), the ADC sampling and the contactor state machine. Updates must therefore be refused unless the contactors are open and the system is INACTIVE.
4. **Supervisor reaction.** The Supervisor MCU runs its own firmware (`superrr`). A controller reboot already happens on a watchdog reset, but check on the desk unit that the Supervisor does nothing drastic during the ~2 s reboot.
5. **Key handling.** The signing key must stay off every device and repo, and key and identity changes must be impossible over the HMI link.

## Options considered

| Option | Verdict |
|---|---|
| **A/B partitions + TBYB, written by the running app (this plan)** | Chosen. Power loss at any point leaves the old image bootable, a bad image rolls back by itself, and no bootloader needs maintaining. |
| Staging area + RAM-resident copy over slot 0 | Simpler build, but a power cut during the copy bricks the controller until it is reflashed over USB. No automatic rollback. |
| RP2350 boot ROM UART boot | Needs boot-strap pins driven by the P4 and loads into SRAM only. Not wired. |
| P4 as USB host speaking PICOBOOT (what picotool does) | Would work, but the P4's USB is not wired to the RP2354B's USB. |
| The P4 toggles RUN/BOOTSEL | Not wired as far as the code shows (only TX/RX). Not needed for this plan. |
| OTP secure boot (ROM-enforced signatures) | Rejected: irreversible, and it would also restrict USB. Signing is enforced in the application for serial OTA only. |

## Flash layout (2 MB RP2354B)

The NVM/LittleFS area stays exactly where it is, so settings and the event log survive the migration.

| Offset | Size | Use |
|---|---|---|
| `0x000000` | 8 KB | Partition table (boot ROM) |
| `0x002000` | 768 KB | Slot A (image, partition 0) |
| `0x0C2000` | 768 KB | Slot B (image, partition 1, linked as A's partner) |
| `0x182000` | 376 KB | Spare |
| `0x1E0000` | 128 KB | NVM / LittleFS (`NVM_FLASH_OFFSET`, unchanged): settings, event log, pack identity, trusted keys. Declare it as a data partition, or leave it unpartitioned. |

The current image is ~150 KB (`build-bluetesla-rev1/bms.bin`), so 768 KB slots leave plenty of room. `PICO_FLASH_SIZE_BYTES` (1.5 MB) stays as the link-time guard. Add a CMake check that the image plus the signature trailer fits in a slot.

## USB / drag-and-drop after migration

USB stays available as a normal update path and as the recovery path. BOOTSEL mode is in ROM, so it can't be lost, and it is never restricted by keys.

- **Drag-and-drop UF2 still works.** With a partition table, the boot ROM writes a UF2 of a family that slots A and B accept (`rp2350-arm-s`) into the slot that is *not* currently booted. It then reboots into it as a flash update, exactly as the serial OTA does.
- **`picotool load` still works,** with or without `-p <partition>`.
- **Every build needs an increasing image version.** On a normal boot the ROM picks between A and B by version, so without one a newer image can lose to an older one on the next reboot. Use the manifest's `fw_version` as the RP2350 image version (`pico_set_binary_version` or the picotool seal option). (Verify the exact A/B tie-break rule in the RP2350 datasheet, "Partition tables / A/B".)
- **Don't drop a pre-migration build onto a migrated board.** Old builds have no TBYB buy, no version, and still read NVM through the translated XIP window (Task 2). They could boot with blank-looking settings, and might reformat LittleFS.
- **Full reset:** `picotool erase`, then reload the partition table and the image. Same as the migration.
- **The boot-time pack check (below) still applies** to anything installed over USB. A wrong image can be flashed, but it won't close the contactors.

## Image manifest and signature

Both are **part of the image file**, so the same `.bin` / `.uf2` is used for USB and for serial OTA. No separate manifest file travels with it.

**Manifest** (`fw_manifest_t`, in its own linker section `.fw_manifest` within the first 4 KB of the image; found by its magic):

| Field | Type | Notes |
|---|---|---|
| magic | `"CKFW"` | |
| manifest_version | u16 | format version |
| fw_version | u32 | monotonically increasing (build counter or build time); also the RP2350 image version for A/B selection |
| git_hash | u32 | short hash |
| build_time | u32 | unix time |
| profile, board | u8, u8 | `BMS_PROFILE`, `BMS_BOARD` |
| chemistry | u8 | `LFP` / `NMC` |
| num_cells | u16 | `NUM_CELLS` |
| cell_presence_mask | u32[4] | `CELL_PRESENCE_MASK` |
| num_modules | u8 | `NUM_MODULE_VOLTAGES` |

**Signature trailer** (appended after the image by the signing tool):

| Field | Type | Notes |
|---|---|---|
| magic | `"CKSG"` | |
| key_id | u8[8] | first 8 bytes of SHA-256(public key) |
| image_len | u32 | bytes covered by the signature (everything before the trailer) |
| signature | u8[64] | Ed25519 over SHA-256(image[0:image_len]) |

- **Tooling:** `tools/sign_image.py <bms.bin> <private key>` appends the trailer, and also emits a signed `.uf2`.
  - CMake runs it when a key path is configured (`BMS_SIGNING_KEY`, outside the repo).
  - Otherwise the build is unsigned: fine for USB, refused over serial.
- **Private key:** never in a repo or on a device. Keep it offline, with a backup. The firmware holds only public keys.
- **No separate image CRC is needed.** The signature covers every byte. The per-frame CRC-16 of the serial protocol stays: it catches line noise per ~200-byte chunk so only that chunk is resent, instead of a whole failed transfer.

## Pack identity and boot-time checks

**Pack identity** (stored in NVM as its own small record, separate from the settings):
- What it holds: chemistry, num_cells, cell_presence_mask, and the profile name for display.
- **Provisioned over USB only:**
  - On the first boot of an image installed over USB, if no identity is stored, the BMS adopts the running image's manifest identity, but only after the boot-time cell check (below) passes.
  - Changing it later needs an explicit USB console command (`identity adopt`) after flashing a different-chemistry image over USB.
- **Serial OTA rule:** the new image's manifest must have **the same chemistry, num_cells and cell_presence_mask** as the stored identity, and the same board revision. Otherwise the result is `IDENTITY_MISMATCH` / `BOARD_MISMATCH`. Changing chemistry or pack layout is a USB-only operation.

**Boot-time cell check (every image, whatever installed it):** before leaving INITIALIZING, i.e. before any contactor test:
- Read the raw BMB channels, including the masked-out ones.
- Every channel **in** `CELL_PRESENCE_MASK` must read a plausible cell voltage for the image's chemistry, e.g. LFP 2.0–3.8 V, NMC 2.5–4.35 V.
- Every channel **outside** the mask must read "no cell" (thresholds to be measured, see Open questions).
- The count of plausible cells must equal `NUM_CELLS`, and the number of responding modules must equal `NUM_MODULE_VOLTAGES`.
- **Chemistry sanity:** any cell above 3.8 V under an LFP image means the image doesn't match the pack.
- **On any mismatch:** raise a FATAL `PACK_CONFIG_MISMATCH` event (appended to the event list) and never close the contactors.
- **After USB provisioning:** if the stored identity differs from the running image's manifest, also go to FAULT until `identity adopt` is run over USB. A wrong image flashed over USB can then never drive a pack it wasn't built for.
- The DESK profile keeps a build flag to relax the check on the bench.

## Trusted keys

- **Our key** is compiled into every build (`VENDOR_PUBLIC_KEY`, 32 bytes) and can't be removed from outside.
- **User keys:** up to 4 extra Ed25519 public keys, each with a label, stored in NVM. They survive firmware updates.
- **Accept rule:** an OTA image is accepted if its trailer's key_id matches a trusted key and the signature verifies.
- **USB-only management** through the USB CDC console (`bms/protocols/cli`):

  ```
  keys list                          # fingerprints + labels
  keys add <label> <64 hex chars>
  keys remove <n>
  keys reset                         # back to our key only
  identity show | identity adopt
  ```

  The HMI serial protocol **never** accepts key or identity changes. Otherwise anyone who could reach the screen could add a key.
- **Anti-downgrade per key:** NVM keeps the highest `fw_version` accepted for each key_id, and a serial OTA with a lower version is refused (`DOWNGRADE`). USB installs are not restricted.
- **Visibility (read-only over HMI):** the running image's signer key_id/label (or "unsigned"), and the trusted key fingerprints.
- **What a user key means:** firmware signed by a user's key runs that user's code, which controls the key store from then on. The "signed by" display makes that visible.

## Transfer protocol (new HMI message types)

The existing frame is `0xFF, len-1, payload…, CRC16` with a payload of at most 255 bytes. All new messages carry the BMS address in byte 1, like the others. Integers are little-endian.

| Msg | Dir | Payload | Reply |
|---|---|---|---|
| `0x10` FW_BEGIN | HMI→BMS | addr, total_size u32 (image + trailer) | `0x90`: status u8, max_chunk u8, slot_offset u32 |
| `0x11` FW_DATA | HMI→BMS | addr, offset u32, data[≤192] | `0x91`: status u8, next_offset u32 |
| `0x12` FW_END | HMI→BMS | addr | `0x92`: status u8, key_id[8], fw_version u32. All checks are done here; see Task 4. |
| `0x13` FW_ACTIVATE | HMI→BMS | addr | `0x93`: status u8, then a reboot after ~100 ms |
| `0x14` FW_ABORT | HMI→BMS | addr | `0x94`: status u8 |

**Status codes:**
- 0 = OK
- 1 = REFUSED_STATE (contactors not open or system not INACTIVE)
- 2 = BOARD_MISMATCH (manifest board ≠ this board)
- 3 = TOO_BIG
- 4 = BAD_OFFSET
- 5 = FLASH_ERROR
- 6 = BAD_IMAGE (no manifest or trailer, or a length mismatch)
- 7 = NOT_PARTITIONED (still on the old layout)
- 8 = BUSY
- 9 = UNKNOWN_KEY (trailer key_id not trusted)
- 10 = BAD_SIGNATURE
- 11 = IDENTITY_MISMATCH (chemistry / num_cells / cell mask ≠ stored pack identity)
- 12 = DOWNGRADE (fw_version below the highest accepted for that key)
- 13 = NOT_PROVISIONED (no pack identity stored yet; provision over USB first)

**Transfer rules:**
- **Stop-and-wait:** the HMI sends the next chunk only after `next_offset` comes back. A repeated or out-of-order chunk is answered with the expected offset, so retries are idempotent.
- **Flash writes:** the BMS fills a 4 KB RAM sector buffer and writes it as one erase plus 16 page programs. Each runs in its own `flash_safe_execute` with a timeout, the same as NVM writes, so no single stall exceeds ~100 ms and the 5 s watchdog stays fed.
- **Timeout:** with no traffic for 30 s the session aborts, and the inactive slot is simply left invalid.
- **Throughput:** about 800 chunks for 150 KB. At ~4.5 ms on the wire plus the loop and reply time, expect **~20–40 s** per update, plus well under a second for the signature check.

## Version and identity registers

The controller has no version string today. These registers expose the manifest and the provisioning state. Registers 62–65 are now taken by the isolation monitor, so this block starts at **70**:

- `HMI_REG_FW_VERSION` (70, uint32): manifest `fw_version`
- `HMI_REG_FW_HASH` (71, uint32): git short hash
- `HMI_REG_FW_BUILD_TIME` (72, uint32): unix time at build
- `HMI_REG_FW_PROFILE_BOARD` (73, uint16): `BMS_PROFILE << 8 | BMS_BOARD`
- `HMI_REG_FW_SLOT` (74, uint8): the running partition, and whether the image is still on trial
- `HMI_REG_FW_SIGNER` (75, uint64): key_id of the running image's signer, 0 = unsigned
- `HMI_REG_PACK_IDENTITY` (76, uint32): chemistry, num_cells and a mask checksum of the stored identity, 0 = not provisioned

The manifest struct is the source of truth. CMake fills it, and also writes a sidecar `bms.fwinfo.json` for the HMI build.

---

## Task 1: Manifest and version / identity registers (Pico + HMI)

**Files:**
- Create: `bms/app/fw_manifest.{c,h}`: the `fw_manifest_t` in section `.fw_manifest`, filled from build-time defines.
- Modify: `CMakeLists.txt`: git hash, build time and fw_version as defines; the linker section; write `bms.fwinfo.json`.
- Modify: `bms/protocols/hmi_serial/hmi_serial.h` (registers 70–76)
- Modify: `bms/protocols/hmi_serial/hmi_serial.c` (availability + append)
- Modify (HMI repo): `components/hmi_bms/{hmi_bms.h,hmi_bms.cpp,sensor.py,text_sensor.py}`, `bms_core.yaml` (a "BMS Firmware" text sensor such as `v123 a1b2c3d BLUETESLA/REV1 signed: CellKeeper`)

- [ ] Step 1: Build the manifest at configure/build time (re-run on every build so the hash stays current). Host test: it's found by magic within the first 4 KB of `bms.bin`.
- [ ] Step 2: Serve registers 70–76. Old HMIs simply never ask for them.
- [ ] Step 3: Decode them in `hmi_bms` and publish a text sensor.
- [ ] Step 4: Host test: the register encoding round-trips (in `tests/`).

This ships on its own, under the current layout, and answers "what is running?" straight away.

## Task 1b: Boot-time pack check and pack identity

**Files:** `bms/app/pack_check.{c,h}` (new), `bms/drivers/bmb3y/bmb3y.c` (raw channel read incl. masked-out channels), `bms/app/state_machines/system.c` (stay in INITIALIZING → FAULT on mismatch), `bms/drivers/chip/nvm.c` (identity record), `bms/sys/events/events.h` (append `PACK_CONFIG_MISMATCH`), `bms/protocols/cli/cli.c` (`identity show|adopt`).

- [ ] Step 1: Pure check function, host-tested: cells in/out of the mask, the count, module count, chemistry voltage windows, and LFP-with-NMC-voltages. Include ck0's and ck1's real masks as cases.
- [ ] Step 2: Run it once BMB data is fresh, before the system can leave INITIALIZING. On failure: FATAL event and no contactor activity.
- [ ] Step 3: Identity record. Adopt it on the first boot of a USB-installed image if empty and the check passed. On a mismatch with the running manifest, FAULT until `identity adopt` over USB.
- [ ] Step 4: Ship this on its own (USB/Pi), before any OTA work. It protects against a wrong image being flashed by USB today.

## Task 2: Make NVM reads translation-safe

**Files:** Modify `bms/drivers/chip/nvm.c` (`FLASH_MMAP_ADDR` and every `memcpy` from `XIP_BASE + …`).

- [ ] Step 1: Read through the untranslated XIP alias (`XIP_NOCACHE_NOALLOC_NOTRANSLATE_BASE + NVM_FLASH_OFFSET`). Check the macro name in pico-sdk 2.2.0 `hardware/regs/addressmap.h`.
- [ ] Step 2: Flash it on the desk board using the current, unpartitioned layout. Settings and the log must read back identically.

Ship this before Task 3. It is harmless on the old layout and required on the new one.

## Task 3: Partition table and slot-aware build

**Files:**
- Create: `bms/config/partition_table.json` (the layout above; A and B as a linked pair, both marked as images, plus an NVM data partition or a reserved gap)
- Modify: `CMakeLists.txt`. Embed the table for the migration image (`pico_embed_pt_in_binary`), and produce two outputs:
  - `bms.uf2`: for USB install into slot A.
  - `bms.bin`: the plain image for OTA (signed by the signing step when a key is configured).

  Mark the image for TBYB when sealing (picotool `seal` option, or the SDK's image-def settings). **Follow the RP2350 OTA example in pico-examples (`pico_w/wifi/ota_update`) for the exact CMake calls in SDK 2.2.0.**
- Create: `tools/migrate_to_partitions.md`. One-off USB procedure through the Pis:
  - `picotool partition create`/`load`, then load into slot A.
  - On ck0 use `~/flash_bms_extpullup.sh` (external pull-up board) with the controller serial `559BDEC617328C18`. On ck1 use `--ser 607CD4F76E6454EC`.
  - Never use the Supervisor serials.
  - Verify with `picotool partition info`.

- [ ] Step 1: Write the table JSON, build, and inspect it with `picotool partition info`.
- [ ] Step 2: Desk board: migrate. Check that it boots from slot A, settings are intact (Task 2), and HMI comms work.
- [ ] Step 3: Desk board: load a second build into slot B with picotool and reboot into it. Check both slots boot.
- [ ] Step 4: Give every build an increasing image version (the manifest `fw_version`). Check that drag-and-drop of a newer UF2 lands in the other slot and is still the one booted after a plain power cycle.

## Task 4: Firmware receiver in the BMS

**Files:**
- Create: `bms/protocols/hmi_serial/hmi_fw_update.{c,h}`. Session state, sector buffer, flash writer, the image checks (SHA-256 via `pico_sha256`, hardware-accelerated; Ed25519 via Monocypher), and the reboot.
- Create: `vendor/monocypher/` (verify-only subset).
- Modify: `bms/protocols/hmi_serial/hmi_serial.c`. Dispatch messages 0x10–0x14, and send the replies from `hmi_fw_update`.
- Modify: `CMakeLists.txt`. Link `pico_sha256` and `pico_bootrom`.

- [ ] Step 1: **FW_BEGIN checks.** Refuse unless all of these hold:
  - The system state is INACTIVE and the contactor state is OPEN.
  - A pack identity is stored (else NOT_PROVISIONED).
  - Size ≤ slot size.
  - Running from a partition. Find the inactive slot with `rom_get_partition_table_info` / `rom_pick_ab_partition`.
- [ ] Step 2: **While a session is open:**
  - Hold the contactor request at OPEN, and refuse system requests (Run, Calibrate…).
  - Raise a new `INFO_FW_UPDATE_IN_PROGRESS` event. Append it at the end of the event table, so older HMIs keep their indices.
- [ ] Step 3: **FW_DATA:**
  - Copy into the sector buffer. When a sector is full, erase it and program 16 pages, each in its own `flash_safe_execute`.
  - A duplicate offset is a no-op (answer `next_offset`); a gap is answered with BAD_OFFSET.
- [ ] Step 4: **FW_END:** after flushing the last sector (padded with 0xFF), check in this order, reading the slot back through the untranslated XIP alias:
  1. The trailer exists, and the manifest is found (else BAD_IMAGE).
  2. key_id is trusted (else UNKNOWN_KEY).
  3. SHA-256 (hardware) of image[0:image_len], then **Ed25519 verify** (else BAD_SIGNATURE).
  4. Manifest board = this board (else BOARD_MISMATCH).
  5. Manifest chemistry, num_cells and cell_presence_mask = stored pack identity (else IDENTITY_MISMATCH).
  6. fw_version ≥ the highest accepted for that key (else DOWNGRADE).

  Only then is the session marked activatable. Reply with key_id and fw_version.
- [ ] Step 5: **FW_ACTIVATE:**
  - Only after a successful FW_END.
  - Save NVM (`nvm_schedule_save_persistent_fast` already exists), wait for it to finish, then reply.
  - Then `rom_reboot(REBOOT2_FLAG_REBOOT_TYPE_FLASH_UPDATE, 100, slot_flash_addr, 0)`.
- [ ] Step 6: **Session timeout and FW_ABORT:** drop the session and clear the event. The inactive slot stays as written, but it is never activated.
- [ ] Step 7: Host tests (CMocka, flash layer mocked):
  - chunk sequencing, duplicates, gaps, the size limit and the state refusal;
  - every FW_END refusal, using test keys and images signed by the tool;
  - a flipped bit anywhere in the image → BAD_SIGNATURE;
  - an LFP image onto an NMC identity → IDENTITY_MISMATCH.
- [ ] Step 8: After a successful activation and buy, record the new highest fw_version for that key.

## Task 4b: Trusted-key store and USB console commands

**Files:** `bms/app/keys.{c,h}` (new: compiled-in vendor key, NVM record for up to 4 user keys), `bms/protocols/cli/cli.c`, `tools/sign_image.py`, `tools/gen_key.py`.

- [ ] Step 1: Key store with host tests: add / remove / reset / limit, lookup by key_id, and persistence across a simulated reboot.
- [ ] Step 2: USB console commands `keys list|add|remove|reset`. Make sure no HMI serial message reaches them (test).
- [ ] Step 3: The signing tool: append the trailer, emit a signed `.uf2`, and add the optional `BMS_SIGNING_KEY` CMake hook. Document key generation and backup.
- [ ] Step 4: Expose the signer and the trusted key fingerprints read-only to the HMI (register 75 / a list read).

## Task 5: Commit ("buy") logic in the BMS

**Files:** Modify `bms/app/state_machines/system.c` (or `init.c`) and `hmi_fw_update.c`.

- [ ] Step 1: At boot, find out whether this image is on trial: the boot type is a flash-update boot, or the image is TBYB-flagged and not yet bought. Expose this in `HMI_REG_FW_SLOT`.
- [ ] Step 2: Call `rom_explicit_buy()` only once all of these hold:
  - The system SM has left INITIALIZING successfully (it already waits 10 s for the Supervisor), which now includes the boot-time pack check.
  - The HMI has made at least one valid request.
  - The BMBs and the ADCs report.

  This must happen before the boot ROM's TBYB watchdog runs out (~16.7 s). Check how that interacts with the existing `watchdog_enable(5000)` in `drivers/chip/watchdog.c`. It may need to start later, or be fed during the buy wait.
- [ ] Step 3: Bench test with a deliberately broken image (e.g. one that hangs in `bms_init`). It must reboot back into the previous slot by itself, and the HMI must report the old version again.

## Task 6: HMI side, bundling and sender

**Files (CellKeeper-BMS-LCD):**
- Modify: `components/hmi_bms/__init__.py`. New optional config key `bms_firmware: <path to the signed bms.bin>`.
  - At codegen, read the file, find its manifest, and check that a trailer exists. Refuse to build with an unsigned image.
  - Emit a `static const uint8_t[]` in flash, plus the size and the manifest fields for display.
  - The P4 app partition is ~7.75 MB, so a 150 KB image is fine.
- Modify: `components/hmi_bms/hmi_bms.{h,cpp}`. An update state machine inside `loop()`:
  - **IDLE → BEGIN → DATA → END → ACTIVATE → VERIFY.**
  - Normal register polling pauses while a session runs.
  - Each step retries up to 3× on timeout (~300 ms) and aborts on any refusal.
  - **VERIFY** waits up to 60 s for the BMS to come back, then reads `HMI_REG_FW_VERSION` and `HMI_REG_FW_SLOT`.
- Modify: `components/hmi_bms/{button.py,text_sensor.py}`.
  - Button **"Update BMS Firmware"**, entity_category config. It works only while Settings Unlock is on and the bundled image's version differs from the running one.
  - Text sensor **"BMS Firmware Update"** showing `up to date` / `update available v124` / `updating 42 %` / `waiting for reboot` / `done v124` / `failed: IDENTITY_MISMATCH`, and so on.
- Modify: `guition-jc1060p470c-esp32-p4.yaml` (ck0) and `shed-guition-jc1060p470c-esp32-p4.yaml` (ck1): `bms_firmware:` pointing at the signed `bms.bin` of `build-bluetesla-rev1-extpullup` and `build-lfp1-rev0` respectively (or at a copied-in `firmware/` directory, so that HMI builds are reproducible).
- Modify: `bms_core.yaml`, `bms_ui_scripts.yaml`: show the progress on the screen as well, and put the button on the System page of `HA-dashboard.yaml`.

- [ ] Step 1: Embed the image and its metadata at codegen.
  - Fail the build if the manifest's profile/board/chemistry doesn't match the per-device `bms_profile`/`bms_board` substitution, so ck1's screen can never carry ck0's image.
  - The BMS enforces this independently.
- [ ] Step 2: State machine plus the button and text sensor.
- [ ] Step 3: Never start by itself. Don't start, or abort, if the contactor state is not OPEN; the BMS checks this too.

## Task 7: Bench test matrix (desk board, `build-desk-rev1`)

- [ ] Normal update: A→B, then B→A. Check the version, the settings and the event log afterwards.
- [ ] USB drag-and-drop and `picotool load` after migration both land in the inactive slot and boot (signed or not).
- [ ] Power cut during DATA, during END and during ACTIVATE. The old image must still boot every time.
- [ ] Bad image (never buys) → automatic rollback.
- [ ] **Each refusal:**
  - wrong board → BOARD_MISMATCH
  - LFP↔NMC or a different cell mask → IDENTITY_MISMATCH
  - unsigned → BAD_IMAGE
  - a user key not added → UNKNOWN_KEY
  - tampered byte → BAD_SIGNATURE
  - older version → DOWNGRADE
  - contactors closed → REFUSED_STATE
- [ ] **User key:** add a key over USB, then OTA an image signed with it → accepted. `keys remove` → the same image is refused.
- [ ] **Boot-time check via USB:** flash an LFP image onto the NMC desk pack over USB → FATAL `PACK_CONFIG_MISMATCH`, and the contactors never move.
- [ ] Unplug one BMB cell tap → mismatch at boot.
- [ ] Watch the Supervisor's behaviour through the reboot (scope the contactor drive lines; no unexpected closure).
- [ ] Two BMS addresses on the bus: only the target responds.
- [ ] Timing: total update time, the signature check time, and the longest single flash stall (log all three).

## Rollout

1. **Tasks 1, 1b and 2** ship as a normal USB/Pi update, with no layout change.
   - On first boot each battery provisions its pack identity from the running image, after the cell check passes.
   - Confirm with `identity show`.
2. **Task 3 migration:** ck0 through `nmc-blue-tesla-pi`, ck1 through `lfp-shed-pi` (controller serials only).
3. **Key store and signing (Task 4b)** ship before the first serial OTA. From then on every release image is signed with our key.
4. **From then on,** update ck0 and ck1 by OTA-ing the HMI and pressing "Update BMS Firmware" with the battery isolated (contactors open). Keep the Pi/USB path as the recovery route; USB is never restricted by keys.

## Open questions

- Does anything on the board let the P4 reset the RP2354B (RUN pin)? Not required for this plan, but it would allow a forced recovery.
- What does the Supervisor do when the controller stops talking for ~2–15 s?
- Do ck1's Rev0 board and stacked flash really match the 2 MB layout? Check with `picotool info -a` on ck1 before its migration.
- **The cell-voltage windows for "no cell" vs "cell present"** on masked-out BMB channels: measure what an unused channel actually reads on the 3Y BMBs before choosing the thresholds.
- **Ed25519 vs ECDSA P-256:** Ed25519 (Monocypher) is smaller and simpler. P-256 would only matter if we ever want the RP2350 ROM to check the same signatures, and we've chosen not to use OTP secure boot.
