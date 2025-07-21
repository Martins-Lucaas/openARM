// [IMPORTS]
import 'dart:convert';
import 'dart:math';
import 'package:camera/camera.dart';
import 'package:flutter/material.dart';
import 'package:web_socket_channel/web_socket_channel.dart';
import 'manual_control_page.dart';

class CameraControlPage extends StatefulWidget {
  final List<CameraDescription> cameras;

  const CameraControlPage({super.key, required this.cameras});

  @override
  State<CameraControlPage> createState() => _CameraControlPageState();
}

class _CameraControlPageState extends State<CameraControlPage>
    with SingleTickerProviderStateMixin {
  late CameraController _controller;
  late Future<void> _initializeControllerFuture;
  late WebSocketChannel _channel;

  double baseRotation = 90;
  bool gripperOpen = true;

  String connectionStatus = 'Conectando...';
  String lastCommand = '{}';
  double? batteryVoltage;

  bool fullScreenMode = false;

  Offset? _tapPosition;
  late AnimationController _tapAnimController;

  @override
  void initState() {
    super.initState();

    _controller = CameraController(
      widget.cameras[0],
      ResolutionPreset.medium,
      enableAudio: false,
    );
    _initializeControllerFuture = _controller.initialize();

    _channel = WebSocketChannel.connect(Uri.parse('ws://192.168.4.1/ws'));

    _channel.stream.listen(
      (message) {
        print('Recebido do ESP: $message');
        setState(() {
          connectionStatus = '🟢 Conectado';
        });

        try {
          final data = jsonDecode(message);
          if (data.containsKey('vbat')) {
            final mv = data['vbat'];
            batteryVoltage = mv / 1000.0;
          }
        } catch (_) {
          print('Mensagem não era JSON válido');
        }
      },
      onError: (error) {
        print('Erro WebSocket: $error');
        setState(() {
          connectionStatus = '🔴 Erro na conexão';
        });
      },
      onDone: () {
        print('WebSocket desconectado');
        setState(() {
          connectionStatus = '🔴 Desconectado';
        });
      },
    );

    _tapAnimController = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 700),
    );
  }

  void _sendServoCommand(int servo, double angle) {
    final command = {
      'servo': servo,
      'angulo': angle.round().clamp(0, 180),
    };
    try {
      final jsonData = jsonEncode(command);
      _channel.sink.add(jsonData);
      setState(() {
        lastCommand = jsonData;
      });
    } catch (e) {
      print('Erro ao enviar dados: $e');
    }
  }

  void _toggleGripper() {
    setState(() {
      gripperOpen = !gripperOpen;
    });
    _sendServoCommand(1, gripperOpen ? 180 : 0);
  }

  void _updateBaseRotation(double value) {
    setState(() {
      baseRotation = value;
    });
    _sendServoCommand(4, baseRotation);
  }

  void _handleTapDown(TapDownDetails details, BoxConstraints constraints) {
    final dx = details.localPosition.dx;
    final dy = details.localPosition.dy;

    setState(() {
      _tapPosition = Offset(dx, dy);
    });
    _tapAnimController.forward(from: 0);

    final px = dx / constraints.maxWidth;
    final py = dy / constraints.maxHeight;
    final xNorm = (px - 0.5) * 2;
    final yNorm = (py - 0.5) * -2;

    const L2 = 4.0;
    const L3 = 4.0;
    const alcanceTotal = L2 + L3;

    final x = xNorm * alcanceTotal;
    final y = yNorm * alcanceTotal;

    final thetaBase = (atan2(x, y) * 180 / pi).clamp(0, 180);
    final r = sqrt(x * x + y * y);

    final cosTheta3 = ((r * r) - (L2 * L2 + L3 * L3)) / (2 * L2 * L3);
    final theta3Rad = acos(cosTheta3.clamp(-1.0, 1.0));
    final theta3 = theta3Rad * 180 / pi;

    final k1 = L2 + L3 * cos(theta3Rad);
    final k2 = L3 * sin(theta3Rad);
    final theta2Rad = atan2(0, r) - atan2(k2, k1);
    final theta2 = theta2Rad * 180 / pi;

    _sendServoCommand(2, theta2);
    _sendServoCommand(3, theta3);
    _sendServoCommand(4, thetaBase.toDouble());

    print('Tap → base: $thetaBase, altura: $theta2, extensão: $theta3');
  }

  @override
  void dispose() {
    _controller.dispose();
    _channel.sink.close();
    _tapAnimController.dispose();
    super.dispose();
  }

  Widget _buildTapAnimation() {
    if (_tapPosition == null) return const SizedBox.shrink();

    return Positioned(
      left: _tapPosition!.dx - 25,
      top: _tapPosition!.dy - 25,
      child: FadeTransition(
        opacity: Tween(begin: 1.0, end: 0.0).animate(_tapAnimController),
        child: Container(
          width: 50,
          height: 50,
          decoration: BoxDecoration(
            shape: BoxShape.circle,
            color: Colors.red.withOpacity(0.4),
            border: Border.all(color: Colors.red, width: 2),
          ),
        ),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    final batteryText = batteryVoltage != null
        ? '🔋 ${batteryVoltage!.toStringAsFixed(1)} V'
        : '';

    return Scaffold(
      appBar: AppBar(
        title: const Text('Controle da Garra'),
        centerTitle: true,
        bottom: PreferredSize(
          preferredSize: const Size.fromHeight(24),
          child: Padding(
            padding: const EdgeInsets.only(bottom: 8.0),
            child: Text(
              '$connectionStatus ${batteryText.isNotEmpty ? '| $batteryText' : ''}',
              style: const TextStyle(fontSize: 14, color: Colors.white70),
            ),
          ),
        ),
      ),
      body: FutureBuilder<void>(
        future: _initializeControllerFuture,
        builder: (context, snapshot) {
          if (snapshot.connectionState != ConnectionState.done) {
            return const Center(child: CircularProgressIndicator());
          }

          return LayoutBuilder(
            builder: (context, constraints) => Stack(
              children: [
                SingleChildScrollView(
                  child: Column(
                    children: [
                      GestureDetector(
                        onTapDown: (details) =>
                            _handleTapDown(details, constraints),
                        child: Stack(
                          children: [
                            AspectRatio(
                              aspectRatio: _controller.value.aspectRatio,
                              child: CameraPreview(_controller),
                            ),
                            _buildTapAnimation(),
                          ],
                        ),
                      ),
                      const SizedBox(height: 10),
                      const Text('Rotação da base'),
                      Slider(
                        value: baseRotation,
                        min: 0,
                        max: 180,
                        divisions: 180,
                        label: baseRotation.round().toString(),
                        onChanged: _updateBaseRotation,
                      ),
                      ElevatedButton.icon(
                        onPressed: _toggleGripper,
                        icon: Icon(gripperOpen
                            ? Icons.pan_tool_alt
                            : Icons.pan_tool),
                        label: Text(
                            gripperOpen ? 'Fechar Garra' : 'Abrir Garra'),
                      ),
                      const SizedBox(height: 10),
                      Text(
                        'Último comando: $lastCommand',
                        style: const TextStyle(fontSize: 13),
                      ),
                      const SizedBox(height: 10),
                      Row(
                        mainAxisAlignment: MainAxisAlignment.center,
                        children: [
                          ElevatedButton.icon(
                            icon: const Icon(Icons.fullscreen),
                            label: const Text("Tela Cheia"),
                            onPressed: () {
                              // Modo fullscreen opcional
                            },
                          ),
                          const SizedBox(width: 10),
                          ElevatedButton.icon(
                            icon: const Icon(Icons.tune),
                            label: const Text("Controle Manual"),
                            onPressed: () {
                              Navigator.push(
                                context,
                                MaterialPageRoute(
                                  builder: (_) =>
                                      ManualControlPage(channel: _channel),
                                ),
                              );
                            },
                          ),
                        ],
                      ),
                      const SizedBox(height: 20),
                    ],
                  ),
                ),
              ],
            ),
          );
        },
      ),
    );
  }
}
