# Security and safety reporting

## Supported versions

Until the project publishes its first qualified release, only the latest commit
on the primary development branch receives fixes. Removed prototype scripts in
Git history are unsupported and must not be restored to control hardware.

After releases begin, this table must be updated as part of every release:

| Version | Supported |
| --- | --- |
| Unreleased development branch | Yes |
| Historical prototype commits | No |

## Report privately

Do not open a public issue for a vulnerability or a behavior that could cause
unexpected motion, defeat a stop command, bypass hardware opt-in, expose a
secret, or enable unauthorized control.

Use the repository host's private security-advisory feature. If that feature is
not enabled, contact the maintainers through the private contact listed in the
repository profile. The project does not currently publish a dedicated security
email address; do not guess one or send sensitive details to the project-team
names in historical documentation.

Include:

- affected commit, version, operating system, Python version, and firmware
  version;
- affected hardware models, if any;
- a minimal reproduction using simulation where possible;
- expected and observed state transitions;
- logs with credentials, serial numbers, usernames, and absolute paths removed;
- whether unexpected physical motion occurred; and
- any temporary mitigation already applied.

Never reproduce a suspected issue on a vehicle with wheels on the ground.

## Response process

Maintainers should acknowledge a report within five business days, assess
severity and immediate containment, and coordinate a fix and disclosure
timeline. A safety-critical report should be treated as release-blocking until a
reviewed fix and regression test exist.

The project will credit reporters who request credit and will not intentionally
disclose identifying information. No monetary bug bounty is currently offered.

## Operational incident

If a vehicle moves unexpectedly:

1. use the independent physical cutoff;
2. disconnect propulsion power only when it is safe to approach;
3. preserve host and firmware logs;
4. label the build and vehicle as quarantined; and
5. do not resume operation until the cause and a verified corrective action are
   documented.

See [docs/safety.md](docs/safety.md) for the broader safety model and
[docs/support.md](docs/support.md) for non-sensitive support requests.
