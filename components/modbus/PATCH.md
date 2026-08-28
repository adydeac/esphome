# Vendored `modbus` — patch contract

This folder is **ESPHome's core `modbus` component, verbatim, plus three
`virtual` keywords.** It is not ours and it is not maintained here.

It exists only so `modbus_tcp` can subclass `ModbusClientHub` and replace the
wire. Delete it the day upstream grows a transport seam.

Deliberately kept separate from `modbus_tcp/CLAUDE.md`: that component is ours
and has a design; this folder is a temporary carrier for a three-line diff. They
have different lifetimes and should not be read as one thing.

## The patch

Three declarations in `modbus.h`, nothing else:

```diff
-  void receive_bytes_();
+  virtual void receive_bytes_();

-  bool send_frame_(const ModbusFrame &frame);
+  virtual bool send_frame_(const ModbusFrame &frame);

-  bool timeout_();
+  virtual bool timeout_();
```

Why each is needed:

- **`receive_bytes_()`** and **`send_frame_()`** are the two wire touchpoints.
  `Modbus::loop()` calls the first directly; the send path calls the second.
  Everything else in the hub is transport-agnostic.
- **`timeout_()`** decides whether a partial response has gone stale, using
  `this->parent_->get_rx_full_threshold()`. There is no UART behind a TCP hub, so
  this faults the moment anything lands in `rx_buffer_` —
  `ModbusClientHub::parse_modbus_frames()` calls it whenever the buffer is
  non-empty, i.e. on the very first response.

Already virtual upstream, no patch needed: `tx_blocked()`,
`tx_delay_remaining()`, `parse_modbus_frames()`, `process_modbus_server_frame()`,
plus `setup()` / `loop()` / `dump_config()` from `Component`.

## Overriding the core component

ESPHome's `external_components` mechanism replaces a core component of the same
name. Because this folder is called `modbus`, `esphome.components.modbus`
resolves here, the namespace stays `esphome::modbus`, and consumer components
compile against it unchanged.

```yaml
external_components:
  - source: { type: local, path: components }
    components: [modbus, modbus_tcp, growatt_master]
```

## Re-vendoring

`../../vendor_modbus.sh [git-ref]` — defaults to `dev`.

The script fetches upstream, copies the folder, and applies the three
substitutions. Each one:

- is skipped if already applied,
- **fails the whole script loudly** if the target string is not found or is not
  unique.

That failure mode is the point. A silent partial patch would produce a copy that
still compiles but no longer has the seam, and the first symptom would be a
crash on a live node.

Run it on every ESPHome update. This project tracks `dev` unpinned with weekly
auto-updates, so this is a recurring chore, not a one-off.

## If the patch stops applying

Upstream changed a signature. Do not guess a replacement — re-derive it:

1. Read the current `modbus.h` / `modbus.cpp`.
2. Confirm the method still exists and still sits on the wire path.
3. Check whether upstream has meanwhile introduced a real transport seam — if so,
   this folder should be deleted rather than repatched.
4. Update the substitution strings in `vendor_modbus.sh`.

## Component churn, as of 2026-08

This component is under active refactor by exciton, in an ordered PR chain.
Landed and pending changes touch exactly the areas this patch depends on:

- `send()` removed in **2026.10.0**
- `send_pdu()`, `send_raw()`, and the `ModbusDevice` shim removed in **2027.2.0**
- `queue_pdu()` gained a `bool` return and `CommandOptions` after 2026.7.4
- **#12421** (interrupt-driven receive event queue, still open) rewrites the
  receive path — when it lands, `receive_bytes_()` may not survive in its current
  form and `modbus_tcp` will need reworking, not rebasing

Watch the PR chain, not just release notes.

## Verifying the vendored copy

```bash
diff -u <upstream>/esphome/components/modbus/modbus.h components/modbus/modbus.h
```

Should show exactly three changed lines. Anything else means the copy has drifted
and should be re-vendored from scratch.
