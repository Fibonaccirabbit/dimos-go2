import { useEffect, useRef, useState } from "react";
import {
  ALL_FORMATS,
  BlobSource,
  BufferTarget,
  Conversion,
  Input,
  Mp4OutputFormat,
  Output,
} from "mediabunny";
import styles from "./StatusBar.module.css";

// MP4 where the browser can encode it (recent Chrome), WebM otherwise.
const MIME_TYPES = [
  "video/mp4;codecs=avc1",
  "video/mp4",
  "video/webm;codecs=vp9",
  "video/webm",
];

function pickMimeType(): string {
  return MIME_TYPES.find((t) => MediaRecorder.isTypeSupported(t)) ?? "";
}

function stamp(date: Date): string {
  const pad = (n: number) => String(n).padStart(2, "0");
  return `${date.getFullYear()}${pad(date.getMonth() + 1)}${pad(date.getDate())}-` +
    `${pad(date.getHours())}${pad(date.getMinutes())}${pad(date.getSeconds())}`;
}

function download(blob: Blob, name: string): void {
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 10_000);
}

/** Chrome's MediaRecorder writes fragmented MP4 whose header duration is 0, which
 * QuickTime/Preview show as a frozen frame. Remux (no re-encode) into a regular MP4
 * with the index up front; fall back to the raw recording if that fails. */
async function playableMp4(blob: Blob): Promise<Blob> {
  const input = new Input({ source: new BlobSource(blob), formats: ALL_FORMATS });
  const target = new BufferTarget();
  const output = new Output({ format: new Mp4OutputFormat({ fastStart: "in-memory" }), target });
  const conversion = await Conversion.init({ input, output, showWarnings: false });
  if (!conversion.isValid) throw new Error("recording cannot be remuxed to MP4");
  await conversion.execute();
  if (target.buffer === null) throw new Error("remux produced no output");
  return new Blob([target.buffer], { type: "video/mp4" });
}

/** Records this cockpit tab with the browser's own tab capture and downloads
 * the video when stopped (button, or the browser's "stop sharing"). Nothing
 * leaves the machine. */
export function RecordButton({ robotName }: { robotName: string | null }) {
  const [startedAt, setStartedAt] = useState<number | null>(null);
  const [now, setNow] = useState(() => Date.now());
  const [error, setError] = useState<string | null>(null);
  const [saving, setSaving] = useState(false);
  const recorder = useRef<MediaRecorder | null>(null);

  useEffect(() => {
    if (startedAt === null) return;
    const id = setInterval(() => setNow(Date.now()), 500);
    return () => clearInterval(id);
  }, [startedAt]);

  useEffect(() => () => recorder.current?.stop(), []);

  const supported = typeof MediaRecorder !== "undefined" &&
    typeof navigator.mediaDevices?.getDisplayMedia === "function";
  if (!supported) return null;

  const start = async (): Promise<void> => {
    setError(null);
    let stream: MediaStream;
    try {
      stream = await navigator.mediaDevices.getDisplayMedia({
        video: { frameRate: 30, displaySurface: "browser" },
        audio: false,
        // Chrome: offer this tab first and allow capturing it.
        preferCurrentTab: true,
        selfBrowserSurface: "include",
      } as DisplayMediaStreamOptions);
    } catch {
      return; // The operator cancelled the share prompt.
    }
    const mimeType = pickMimeType();
    const rec = new MediaRecorder(stream, mimeType ? { mimeType, videoBitsPerSecond: 8e6 } : {});
    const chunks: Blob[] = [];
    const began = new Date();
    rec.ondataavailable = (e) => {
      if (e.data.size > 0) chunks.push(e.data);
    };
    rec.onstop = () => {
      stream.getTracks().forEach((t) => t.stop());
      recorder.current = null;
      setStartedAt(null);
      const type = rec.mimeType || "video/webm";
      const who = (robotName ?? "cockpit").replace(/[^A-Za-z0-9_-]+/g, "-");
      const name = `dimos-${who}-${stamp(began)}`;
      if (chunks.length === 0) {
        setError("recording was empty");
        return;
      }
      const raw = new Blob(chunks, { type });
      setSaving(true);
      void playableMp4(raw)
        .then((mp4) => download(mp4, `${name}.mp4`))
        .catch(() => {
          // Still save what was captured; Chrome and VLC play the raw file.
          download(raw, `${name}.${type.startsWith("video/mp4") ? "mp4" : "webm"}`);
          setError("saved raw recording (QuickTime may not play it)");
        })
        .finally(() => setSaving(false));
    };
    // Ending the share from the browser's own bar also finishes the file.
    stream.getVideoTracks()[0]?.addEventListener("ended", () => {
      if (rec.state !== "inactive") rec.stop();
    });
    rec.start(1000);
    recorder.current = rec;
    setNow(began.getTime());
    setStartedAt(began.getTime());
  };

  const stop = (): void => {
    if (recorder.current !== null && recorder.current.state !== "inactive") recorder.current.stop();
  };

  if (saving) {
    return (
      <button type="button" className={styles.action} data-testid="record" disabled>
        saving…
      </button>
    );
  }
  if (startedAt !== null) {
    const secs = Math.max(0, Math.floor((now - startedAt) / 1000));
    const clock = `${Math.floor(secs / 60)}:${String(secs % 60).padStart(2, "0")}`;
    return (
      <button
        type="button"
        className={styles.recording}
        data-testid="record"
        title="Stop and download the recording"
        onClick={stop}
      >
        ● REC {clock} · stop
      </button>
    );
  }
  return (
    <>
      <button
        type="button"
        className={styles.action}
        data-testid="record"
        title="Record this cockpit tab to a video file"
        onClick={() => void start()}
      >
        ● record
      </button>
      {error !== null && <span className={styles.error}>{error}</span>}
    </>
  );
}
