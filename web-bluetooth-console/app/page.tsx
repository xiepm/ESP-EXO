"use client";

import { FormEvent, KeyboardEvent, useCallback, useEffect, useMemo, useRef, useState } from "react";

const DEVICE_NAME = "RobStride-CAN";
const SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
const RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
const TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";

type BluetoothDeviceLike = {
  name?: string;
  gatt?: {
    connected: boolean;
    connect: () => Promise<BluetoothRemoteGATTServerLike>;
    disconnect: () => void;
  };
  addEventListener: (type: "gattserverdisconnected", listener: () => void) => void;
  removeEventListener: (type: "gattserverdisconnected", listener: () => void) => void;
};

type BluetoothRemoteGATTServerLike = {
  getPrimaryService: (uuid: string) => Promise<BluetoothRemoteGATTServiceLike>;
};

type BluetoothRemoteGATTServiceLike = {
  getCharacteristic: (uuid: string) => Promise<BluetoothRemoteGATTCharacteristicLike>;
};

type BluetoothRemoteGATTCharacteristicLike = {
  value?: DataView;
  startNotifications: () => Promise<BluetoothRemoteGATTCharacteristicLike>;
  stopNotifications?: () => Promise<BluetoothRemoteGATTCharacteristicLike>;
  writeValue?: (value: BufferSource) => Promise<void>;
  writeValueWithoutResponse?: (value: BufferSource) => Promise<void>;
  addEventListener: (type: "characteristicvaluechanged", listener: (event: Event) => void) => void;
  removeEventListener: (type: "characteristicvaluechanged", listener: (event: Event) => void) => void;
};

type BluetoothNavigator = Navigator & {
  bluetooth?: {
    requestDevice: (options: {
      filters: Array<{ name: string }>;
      optionalServices: string[];
    }) => Promise<BluetoothDeviceLike>;
  };
};

type SerialPortInfoLike = {
  usbVendorId?: number;
  usbProductId?: number;
};

type SerialPortLike = {
  readable: ReadableStream<Uint8Array> | null;
  writable: WritableStream<Uint8Array> | null;
  open: (options: { baudRate: number; dataBits?: number; stopBits?: number; parity?: "none" | "even" | "odd"; flowControl?: "none" | "hardware" }) => Promise<void>;
  close: () => Promise<void>;
  getInfo: () => SerialPortInfoLike;
};

type SerialNavigator = Navigator & {
  serial?: {
    requestPort: () => Promise<SerialPortLike>;
  };
};

type Direction = "rx" | "tx" | "system" | "error";
type Entry = {
  id: number;
  at: Date;
  direction: Direction;
  bytes?: Uint8Array;
  message?: string;
};
type SendMode = "text" | "hex";
type LineEnding = "none" | "cr" | "lf" | "crlf";
type Transport = "ble" | "serial";

const BAUD_RATES = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600];

const encoder = new TextEncoder();
const decoder = new TextDecoder("utf-8", { fatal: false });

function toHex(bytes: Uint8Array) {
  return Array.from(bytes, (byte) => byte.toString(16).padStart(2, "0").toUpperCase()).join(" ");
}

function toVisibleText(bytes: Uint8Array) {
  return decoder
    .decode(bytes)
    .replaceAll("\r", "␍")
    .replaceAll("\n", "␊\n")
    .replace(/[\x00-\x08\x0B\x0C\x0E-\x1F\x7F]/g, "·");
}

function parseHex(input: string) {
  const normalized = input.replace(/0x/gi, "").replace(/[\s,;:_-]+/g, "").trim();
  if (!normalized) return new Uint8Array();
  if (!/^[0-9a-f]+$/i.test(normalized)) throw new Error("HEX 中只能包含 0–9 和 A–F");
  if (normalized.length % 2 !== 0) throw new Error("HEX 字节必须由两个字符组成，例如 0A FF");
  return Uint8Array.from(normalized.match(/.{2}/g)!.map((pair) => Number.parseInt(pair, 16)));
}

function timestamp(date: Date) {
  return `${date.toLocaleTimeString("zh-CN", { hour12: false })}.${String(date.getMilliseconds()).padStart(3, "0")}`;
}

export default function Home() {
  const [bleSupported, setBleSupported] = useState<boolean | null>(null);
  const [serialSupported, setSerialSupported] = useState<boolean | null>(null);
  const [transport, setTransport] = useState<Transport>("ble");
  const [connection, setConnection] = useState<"idle" | "connecting" | "connected">("idle");
  const [deviceName, setDeviceName] = useState(DEVICE_NAME);
  const [serialLabel, setSerialLabel] = useState("USB 串口");
  const [baudRate, setBaudRate] = useState(115200);
  const [entries, setEntries] = useState<Entry[]>([]);
  const [sendMode, setSendMode] = useState<SendMode>("text");
  const [lineEnding, setLineEnding] = useState<LineEnding>("crlf");
  const [payload, setPayload] = useState("");
  const [sending, setSending] = useState(false);
  const [detailMode, setDetailMode] = useState<"both" | "text" | "hex">("both");
  const [autoScroll, setAutoScroll] = useState(true);
  const [error, setError] = useState("");

  const deviceRef = useRef<BluetoothDeviceLike | null>(null);
  const rxRef = useRef<BluetoothRemoteGATTCharacteristicLike | null>(null);
  const txRef = useRef<BluetoothRemoteGATTCharacteristicLike | null>(null);
  const serialPortRef = useRef<SerialPortLike | null>(null);
  const serialReaderRef = useRef<ReadableStreamDefaultReader<Uint8Array> | null>(null);
  const serialReadTaskRef = useRef<Promise<void> | null>(null);
  const serialClosingRef = useRef(false);
  const nextId = useRef(1);
  const logEndRef = useRef<HTMLDivElement | null>(null);

  const addEntry = useCallback((entry: Omit<Entry, "id" | "at">) => {
    setEntries((current) => [
      ...current.slice(-499),
      { ...entry, id: nextId.current++, at: new Date() },
    ]);
  }, []);

  const resetConnection = useCallback(() => {
    rxRef.current = null;
    txRef.current = null;
    serialReaderRef.current = null;
    serialReadTaskRef.current = null;
    setConnection("idle");
  }, []);

  const handleDisconnected = useCallback(() => {
    resetConnection();
    addEntry({ direction: "system", message: "设备已断开，可点击重新连接。" });
  }, [addEntry, resetConnection]);

  const handleNotification = useCallback((event: Event) => {
    const characteristic = event.target as unknown as BluetoothRemoteGATTCharacteristicLike;
    if (!characteristic.value) return;
    const view = characteristic.value;
    const bytes = new Uint8Array(view.buffer.slice(view.byteOffset, view.byteOffset + view.byteLength));
    addEntry({ direction: "rx", bytes });
  }, [addEntry]);

  useEffect(() => {
    const capabilityTimer = window.setTimeout(() => {
      setBleSupported(Boolean((navigator as BluetoothNavigator).bluetooth));
      setSerialSupported(Boolean((navigator as SerialNavigator).serial));
    }, 0);
    return () => {
      window.clearTimeout(capabilityTimer);
      const device = deviceRef.current;
      if (device) device.removeEventListener("gattserverdisconnected", handleDisconnected);
      const tx = txRef.current;
      if (tx) tx.removeEventListener("characteristicvaluechanged", handleNotification);
    };
  }, [handleDisconnected, handleNotification]);

  useEffect(() => {
    if (autoScroll) logEndRef.current?.scrollIntoView({ behavior: "smooth", block: "end" });
  }, [entries, autoScroll]);

  async function connectBluetooth() {
    const bluetooth = (navigator as BluetoothNavigator).bluetooth;
    if (!bluetooth) return;
    setError("");
    setConnection("connecting");
    addEntry({ direction: "system", message: "正在打开设备选择器…" });

    try {
      const device = await bluetooth.requestDevice({
        filters: [{ name: DEVICE_NAME }],
        optionalServices: [SERVICE_UUID],
      });
      deviceRef.current = device;
      setDeviceName(device.name || DEVICE_NAME);
      device.addEventListener("gattserverdisconnected", handleDisconnected);

      const server = await device.gatt!.connect();
      const service = await server.getPrimaryService(SERVICE_UUID);
      const [rx, tx] = await Promise.all([
        service.getCharacteristic(RX_UUID),
        service.getCharacteristic(TX_UUID),
      ]);

      rxRef.current = rx;
      txRef.current = tx;
      tx.addEventListener("characteristicvaluechanged", handleNotification);
      await tx.startNotifications();
      setConnection("connected");
      addEntry({ direction: "system", message: `已连接 ${device.name || DEVICE_NAME}，TX 通知已开启。` });
    } catch (caught) {
      resetConnection();
      const message = caught instanceof Error ? caught.message : "连接失败";
      if (message.toLowerCase().includes("cancel")) {
        addEntry({ direction: "system", message: "已取消设备选择。" });
      } else {
        setError(message);
        addEntry({ direction: "error", message: `连接失败：${message}` });
      }
    }
  }

  async function readSerial(port: SerialPortLike) {
    try {
      while (port.readable && !serialClosingRef.current) {
        const reader = port.readable.getReader();
        serialReaderRef.current = reader;
        try {
          while (!serialClosingRef.current) {
            const { value, done } = await reader.read();
            if (done) break;
            if (value?.length) addEntry({ direction: "rx", bytes: new Uint8Array(value) });
          }
        } finally {
          reader.releaseLock();
          if (serialReaderRef.current === reader) serialReaderRef.current = null;
        }
      }
    } catch (caught) {
      if (!serialClosingRef.current) {
        const message = caught instanceof Error ? caught.message : "串口读取失败";
        setError(message);
        addEntry({ direction: "error", message: `串口读取失败：${message}` });
      }
    } finally {
      if (!serialClosingRef.current) {
        serialPortRef.current = null;
        resetConnection();
        addEntry({ direction: "system", message: "串口已断开，可点击重新连接。" });
      }
    }
  }

  async function connectSerial() {
    const serial = (navigator as SerialNavigator).serial;
    if (!serial) return;
    setError("");
    setConnection("connecting");
    addEntry({ direction: "system", message: "正在打开串口选择器…" });

    try {
      const port = await serial.requestPort();
      await port.open({ baudRate, dataBits: 8, stopBits: 1, parity: "none", flowControl: "none" });
      const info = port.getInfo();
      const identifiers = [
        info.usbVendorId !== undefined ? `VID ${info.usbVendorId.toString(16).padStart(4, "0").toUpperCase()}` : "",
        info.usbProductId !== undefined ? `PID ${info.usbProductId.toString(16).padStart(4, "0").toUpperCase()}` : "",
      ].filter(Boolean).join(" · ");

      serialPortRef.current = port;
      serialClosingRef.current = false;
      setSerialLabel(identifiers || "USB 串口");
      setConnection("connected");
      addEntry({ direction: "system", message: `串口已连接，波特率 ${baudRate}${identifiers ? `，${identifiers}` : ""}。` });
      const readTask = readSerial(port);
      serialReadTaskRef.current = readTask;
      void readTask;
    } catch (caught) {
      resetConnection();
      const message = caught instanceof Error ? caught.message : "串口连接失败";
      if (message.toLowerCase().includes("cancel") || message.toLowerCase().includes("no port selected")) {
        addEntry({ direction: "system", message: "已取消串口选择。" });
      } else {
        setError(message);
        addEntry({ direction: "error", message: `串口连接失败：${message}` });
      }
    }
  }

  async function disconnectSerial() {
    const port = serialPortRef.current;
    if (!port) {
      resetConnection();
      return;
    }
    serialClosingRef.current = true;
    try {
      await serialReaderRef.current?.cancel();
      await serialReadTaskRef.current;
      await port.close();
    } catch (caught) {
      const message = caught instanceof Error ? caught.message : "串口关闭失败";
      addEntry({ direction: "error", message: `串口关闭失败：${message}` });
    } finally {
      serialPortRef.current = null;
      resetConnection();
      serialClosingRef.current = false;
      addEntry({ direction: "system", message: "串口已断开。" });
    }
  }

  async function connectSelected() {
    if (transport === "ble") await connectBluetooth();
    else await connectSerial();
  }

  async function disconnect() {
    if (transport === "serial") {
      await disconnectSerial();
      return;
    }
    const device = deviceRef.current;
    if (device?.gatt?.connected) device.gatt.disconnect();
    else resetConnection();
  }

  async function writeBytes(bytes: Uint8Array) {
    if (transport === "serial") {
      const port = serialPortRef.current;
      if (!port?.writable) throw new Error("串口当前不可写");
      const writer = port.writable.getWriter();
      try {
        await writer.write(bytes);
      } finally {
        writer.releaseLock();
      }
      return;
    }

    if (!rxRef.current) throw new Error("BLE RX 特征尚未连接");
    const writer = rxRef.current.writeValueWithoutResponse?.bind(rxRef.current)
      ?? rxRef.current.writeValue?.bind(rxRef.current);
    if (!writer) throw new Error("浏览器未提供可用的 BLE 写入方法");
    for (let offset = 0; offset < bytes.length; offset += 180) {
      await writer(bytes.slice(offset, offset + 180));
    }
  }

  function buildPayload() {
    if (sendMode === "hex") return parseHex(payload);
    const endings: Record<LineEnding, string> = { none: "", cr: "\r", lf: "\n", crlf: "\r\n" };
    return encoder.encode(payload + endings[lineEnding]);
  }

  async function send(event?: FormEvent) {
    event?.preventDefault();
    if (connection !== "connected" || !payload.trim() || sending) return;
    setError("");
    setSending(true);
    try {
      const bytes = buildPayload();
      await writeBytes(bytes);
      addEntry({ direction: "tx", bytes });
      if (sendMode === "text") setPayload("");
    } catch (caught) {
      const message = caught instanceof Error ? caught.message : "发送失败";
      setError(message);
      addEntry({ direction: "error", message: `发送失败：${message}` });
    } finally {
      setSending(false);
    }
  }

  async function sendQuick(command: string) {
    setSendMode("text");
    setLineEnding("crlf");
    setPayload(command);
    if (connection !== "connected" || sending) return;
    setSending(true);
    try {
      const bytes = encoder.encode(`${command}\r\n`);
      await writeBytes(bytes);
      addEntry({ direction: "tx", bytes });
      setPayload("");
    } catch (caught) {
      const message = caught instanceof Error ? caught.message : "发送失败";
      setError(message);
      addEntry({ direction: "error", message: `发送失败：${message}` });
    } finally {
      setSending(false);
    }
  }

  function handleKeyDown(event: KeyboardEvent<HTMLTextAreaElement>) {
    if ((event.ctrlKey || event.metaKey) && event.key === "Enter") {
      event.preventDefault();
      void send();
    }
  }

  async function copyLog() {
    const text = entries.map((entry) => {
      const prefix = `[${timestamp(entry.at)}] ${entry.direction.toUpperCase()}`;
      if (entry.message) return `${prefix} ${entry.message}`;
      return `${prefix} ${toVisibleText(entry.bytes!)} | ${toHex(entry.bytes!)}`;
    }).join("\n");
    await navigator.clipboard.writeText(text);
    addEntry({ direction: "system", message: "日志已复制到剪贴板。" });
  }

  const traffic = useMemo(() => entries.filter((entry) => entry.bytes), [entries]);
  const bytesReceived = useMemo(() => traffic.filter((entry) => entry.direction === "rx").reduce((sum, entry) => sum + entry.bytes!.length, 0), [traffic]);
  const bytesSent = useMemo(() => traffic.filter((entry) => entry.direction === "tx").reduce((sum, entry) => sum + entry.bytes!.length, 0), [traffic]);
  const isConnected = connection === "connected";
  const selectedSupported = transport === "ble" ? bleSupported : serialSupported;
  const transportName = transport === "ble" ? "蓝牙" : "串口";

  return (
    <main className="app-shell">
      <header className="topbar">
        <div className="brand-block">
          <div className="brand-mark" aria-hidden="true"><span /></div>
          <div>
            <p className="eyebrow">ESP-EXO · BLE / SERIAL</p>
            <h1>RobStride 通信控制台</h1>
          </div>
        </div>
        <div className={`connection-pill ${connection}`}>
          <span className="status-dot" />
          {connection === "connected" ? `${transportName}在线` : connection === "connecting" ? `正在连接${transportName}` : "未连接"}
        </div>
      </header>

      {selectedSupported === false && (
        <section className="browser-alert" role="alert">
          <strong>当前浏览器不支持 Web {transport === "ble" ? "Bluetooth" : "Serial"}</strong>
          <span>请在 Windows 上使用最新版 Chrome / Edge，并通过 localhost 打开本页。</span>
        </section>
      )}

      <section className="hero-panel">
        <div className="hero-copy">
          <p className="section-kicker">无线与有线调试链路</p>
          <h2>连接 ESP32，直接收发原始报文</h2>
          <p>可通过 BLE Nordic UART Service 或 USB 串口与 ESP32 直连。选择通信方式后，即可持续读取、写入并记录文本或 HEX 报文。</p>
        </div>
        <div className="device-card">
          <div className="transport-switch" aria-label="通信方式">
            <button className={transport === "ble" ? "active" : ""} disabled={connection !== "idle"} onClick={() => setTransport("ble")}>蓝牙</button>
            <button className={transport === "serial" ? "active" : ""} disabled={connection !== "idle"} onClick={() => setTransport("serial")}>USB 串口</button>
          </div>
          <div className={`device-icon ${transport}`} aria-hidden="true"><span>{transport === "ble" ? "BT" : "USB"}</span></div>
          <div className="device-info">
            <span>{transport === "ble" ? "目标设备" : "串口设备"}</span>
            <strong>{transport === "ble" ? deviceName : serialLabel}</strong>
            <small>{isConnected ? (transport === "ble" ? "NUS 服务已就绪" : `${baudRate} baud · 8-N-1`) : "等待浏览器授权"}</small>
          </div>
          {transport === "serial" && !isConnected && (
            <label className="baud-control">波特率
              <select value={baudRate} disabled={connection !== "idle"} onChange={(event) => setBaudRate(Number(event.target.value))}>
                {BAUD_RATES.map((rate) => <option value={rate} key={rate}>{rate}</option>)}
              </select>
            </label>
          )}
          {isConnected ? (
            <button className="button secondary" onClick={() => void disconnect()}>断开连接</button>
          ) : (
            <button className="button primary" onClick={() => void connectSelected()} disabled={selectedSupported !== true || connection === "connecting"}>
              {connection === "connecting" ? "连接中…" : transport === "ble" ? "选择蓝牙设备" : "选择串口"}
            </button>
          )}
        </div>
      </section>

      <section className="workspace-grid">
        <div className="terminal-panel panel">
          <div className="panel-header">
            <div>
              <p className="panel-label">实时数据流</p>
              <h3>报文监视器</h3>
            </div>
            <div className="terminal-actions">
              <div className="segmented" aria-label="报文显示模式">
                {(["both", "text", "hex"] as const).map((mode) => (
                  <button key={mode} className={detailMode === mode ? "active" : ""} onClick={() => setDetailMode(mode)}>
                    {{ both: "双视图", text: "文本", hex: "HEX" }[mode]}
                  </button>
                ))}
              </div>
              <button className="icon-button" onClick={() => void copyLog()} disabled={!entries.length} title="复制日志">复制</button>
              <button className="icon-button" onClick={() => setEntries([])} disabled={!entries.length} title="清空日志">清空</button>
            </div>
          </div>

          <div className="terminal" aria-live="polite" aria-label="蓝牙报文日志">
            {entries.length === 0 ? (
              <div className="terminal-empty">
                <div className="pulse-rings"><span /><span /><i /></div>
                <strong>等待数据</strong>
                <p>连接设备后，接收和发送的报文会显示在这里。</p>
              </div>
            ) : entries.map((entry) => (
              <article className={`log-entry ${entry.direction}`} key={entry.id}>
                <time>{timestamp(entry.at)}</time>
                <span className="direction-tag">{entry.direction === "rx" ? "RX" : entry.direction === "tx" ? "TX" : entry.direction === "error" ? "ERR" : "SYS"}</span>
                <div className="log-content">
                  {entry.message ? <p>{entry.message}</p> : (
                    <>
                      {detailMode !== "hex" && <pre className="ascii-line">{toVisibleText(entry.bytes!) || "（空报文）"}</pre>}
                      {detailMode !== "text" && <pre className="hex-line">{toHex(entry.bytes!) || "—"}</pre>}
                    </>
                  )}
                </div>
                {entry.bytes && <span className="byte-count">{entry.bytes.length} B</span>}
              </article>
            ))}
            <div ref={logEndRef} />
          </div>

          <div className="terminal-footer">
            <label className="toggle-row">
              <input type="checkbox" checked={autoScroll} onChange={(event) => setAutoScroll(event.target.checked)} />
              <span className="toggle" />
              自动滚动
            </label>
            <div className="traffic-stats">
              <span><i className="rx-dot" /> RX {bytesReceived} B</span>
              <span><i className="tx-dot" /> TX {bytesSent} B</span>
              <span>{traffic.length} 帧</span>
            </div>
          </div>
        </div>

        <aside className="side-stack">
          <form className="send-panel panel" onSubmit={(event) => void send(event)}>
            <div className="panel-header compact">
              <div>
                <p className="panel-label">{transport === "ble" ? "写入 BLE RX 特征" : "写入串口 TX"}</p>
                <h3>发送报文</h3>
              </div>
              <div className="mode-switch">
                <button type="button" className={sendMode === "text" ? "active" : ""} onClick={() => setSendMode("text")}>文本</button>
                <button type="button" className={sendMode === "hex" ? "active" : ""} onClick={() => setSendMode("hex")}>HEX</button>
              </div>
            </div>

            <label className="field-label" htmlFor="payload">{sendMode === "text" ? "指令或文本" : "十六进制字节"}</label>
            <textarea
              id="payload"
              value={payload}
              onChange={(event) => setPayload(event.target.value)}
              onKeyDown={handleKeyDown}
              placeholder={sendMode === "text" ? "例如：status" : "例如：68 65 6C 70 0D 0A"}
              rows={5}
              spellCheck={false}
            />

            <div className="send-options">
              {sendMode === "text" ? (
                <label>行尾
                  <select value={lineEnding} onChange={(event) => setLineEnding(event.target.value as LineEnding)}>
                    <option value="crlf">CRLF  \r\n</option>
                    <option value="lf">LF  \n</option>
                    <option value="cr">CR  \r</option>
                    <option value="none">无</option>
                  </select>
                </label>
              ) : <span className="format-note">空格、逗号和 0x 前缀均可</span>}
              <span className="shortcut">Ctrl + Enter</span>
            </div>

            {error && <p className="inline-error" role="alert">{error}</p>}
            <button className="button primary send-button" type="submit" disabled={!isConnected || !payload.trim() || sending}>
              <span>发送到设备</span><b aria-hidden="true">↗</b>
            </button>
          </form>

          <section className="quick-panel panel">
            <div className="panel-header compact">
              <div>
                <p className="panel-label">常用调试</p>
                <h3>快捷指令</h3>
              </div>
            </div>
            <div className="quick-grid">
              {["help", "status", "motor A", "motor B"].map((command) => (
                <button key={command} disabled={!isConnected || sending} onClick={() => void sendQuick(command)}>
                  <code>{command}</code><span>发送</span>
                </button>
              ))}
            </div>
            <p className="safety-note"><span>!</span><strong>安全提示</strong> 电机使能或运动命令可能导致机构突然动作，请先架空设备并确认急停可用。</p>
          </section>
        </aside>
      </section>

      <footer>
        <span>RobStride-CAN Web Console</span>
        <span className="uuid-note">BLE NUS · USB SERIAL · 8-N-1</span>
        <span>浏览器与设备点对点通信</span>
      </footer>
    </main>
  );
}
