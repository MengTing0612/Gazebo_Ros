# Stabilize SDK TCP Demo

这个目录里的 `stabilize_sdk_tcp_demo.py` 是一个最小 TCP 通信 demo，用来验证本机程序能否和 Stabilize SDK 推理服务通信。

## 启动顺序

1. 准备 SDK 服务端程序，例如 `retina-server-x86_64` 或 `retina-server-arm`。
2. 在一个终端启动服务端：

```bash
chmod +x ./retina-server-x86_64
./retina-server-x86_64
```

首次启动服务端时，它会按 SDK 文档要求登录网页端账号、激活设备、加载内嵌加密模型，然后监听 `127.0.0.1:9000`。

3. 看到服务端输出 `Server listening on 127.0.0.1:9000` 后，在另一个终端运行 demo：

```bash
python examples/sdk_comm/stabilize_sdk_tcp_demo.py
```

注意这里有两层登录：

- 服务端启动登录：用于网页账号授权、设备激活和模型解密。
- TCP 客户端登录：用于连接本地推理服务，文档默认是 `admin/admin123`。

demo 默认使用 TCP 本地账号 `admin/admin123`。如果 SDK 服务端改过 TCP 账号，可以用 `--username` 和 `--password` 覆盖。

也可以通过 `--user-info` 指定 TCP 登录文件，支持下面这种格式：

```text
user_id:admin
pwd:admin123
```

默认连接地址是 `127.0.0.1:9000`。如果服务端监听在别的机器或端口上，使用：

```bash
python examples/sdk_comm/stabilize_sdk_tcp_demo.py --host 192.168.1.10 --port 9000
```

如果要临时覆盖 TCP 账号密码，可以使用：

```bash
python examples/sdk_comm/stabilize_sdk_tcp_demo.py --username your_account --password your_password
```

## 常见错误

### Connection refused

含义：客户端找到了目标主机，但目标端口没有服务在监听。

优先检查：

- `retina-server-x86_64` 或 `retina-server-arm` 是否已经启动。
- demo 的 `--host` 和 `--port` 是否和服务端一致。
- 如果服务端在另一台机器上，防火墙或网络是否允许访问该端口。

### Timeout

含义：客户端没有在超时时间内完成连接或收到回复。

优先检查：

- 服务端进程是否卡住。
- 模型文件是否加载成功。
- 网络是否可达。
- 必要时增大 `--timeout`。

## 协议格式

demo 使用 UTF-8 编码的 JSON Lines 协议：每次发送一个 JSON 对象，并以换行符结尾。

当前流程是：

1. `login`
2. `resume`
3. `infer`
4. 可选 `logout`
