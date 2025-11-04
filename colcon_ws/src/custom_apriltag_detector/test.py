#!/usr/bin/env python3
import os, pty, select, re, sys

RE_XYZ = re.compile(r"^XYZ:\s*(?P<x>-?\d+(?:\.\d+)?)\s+(?P<y>-?\d+(?:\.\d+)?)\s+(?P<z>-?\d+(?:\.\d+)?)")
CMD = "voxl-inspect-tags"  # shell cmd; no buffering issues with PTY

def main():
    print("Listening to AprilTag positions (X, Y, Z in meters)...\n")
    master_fd, slave_fd = pty.openpty()
    pid = os.fork()
    if pid == 0:
        # child: attach to PTY and exec
        os.setsid()
        os.dup2(slave_fd, 0)  # stdin
        os.dup2(slave_fd, 1)  # stdout
        os.dup2(slave_fd, 2)  # stderr
        os.close(master_fd); os.close(slave_fd)
        os.execvp("/bin/bash", ["/bin/bash", "-lc", CMD])
    else:
        # parent: read from PTY
        os.close(slave_fd)
        buf = ""
        try:
            while True:
                r, _, _ = select.select([master_fd], [], [], 0.5)
                if master_fd in r:
                    chunk = os.read(master_fd, 4096).decode("utf-8", errors="ignore")
                    if not chunk:
                        break
                    buf += chunk.replace("\r", "\n")  # normalize CR to LF
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        line = line.strip()
                        m = RE_XYZ.match(line)
                        if m:
                            x = float(m.group("x")); y = float(m.group("y")); z = float(m.group("z"))
                            horizontal_align = x < 0.2 and x > -0.2
                            ready_to_land = horizontal_align and z < 1.8
                            print(f"Tag position: x={x:.2f} m, y={y:.2f} m, z={z:.2f} m")
                            # if ready_to_land:
                            #     print("Ready to land!")
                            # elif horizontal_align:
                            #     print(f"Horizontally Aligned, move forward")
                            # elif not horizontal_align and  x>=0.2: 
                            #     print(f"Not yet horizontall aligned, move right")
                            # elif not horizontal_align and  x<=-0.2:
                            #     print(f"Not yet horizontall aligned, move left")
        except KeyboardInterrupt:
            pass
        finally:
            os.close(master_fd)
            # Reap child
            try:
                os.waitpid(pid, 0)
            except ChildProcessError:
                pass
            print("\nStopping...")

if __name__ == "__main__":
    main()
