import 'dart:convert';
import 'dart:math';
import 'package:flutter/material.dart';
import 'package:web_socket_channel/web_socket_channel.dart';

class ManualControlPage extends StatefulWidget {
  final WebSocketChannel channel;

  const ManualControlPage({super.key, required this.channel});

  @override
  State<ManualControlPage> createState() => _ManualControlPageState();
}

class _ManualControlPageState extends State<ManualControlPage> {
  final Map<int, double> servoValues = {
    1: 0,
    2: 90,
    3: 90,
    4: 90,
  };

  double? batteryVoltage;
  bool isConnected = false;

  @override
  void initState() {
    super.initState();

    widget.channel.stream.listen(
      (data) {
        setState(() {
          isConnected = true;
        });

        try {
          final decoded = jsonDecode(data);
          if (decoded.containsKey('vbat')) {
            batteryVoltage = decoded['vbat'] / 1000.0;
          }
        } catch (e) {
          print('Error decoding message: $e');
        }
      },
      onDone: () {
        setState(() {
          isConnected = false;
        });
      },
      onError: (err) {
        print('WebSocket error: $err');
        setState(() {
          isConnected = false;
        });
      },
    );
  }

  void _sendServoCommand(int servo, double angle) {
    final command = {
      'servo': servo,
      'angulo': angle.round().clamp(0, 180),
    };
    try {
      widget.channel.sink.add(jsonEncode(command));
    } catch (e) {
      print('Error sending command: $e');
    }
  }

  void _resetAllServos() {
    setState(() {
      servoValues.updateAll((key, value) => 0);
    });
    servoValues.forEach((servo, angle) {
      _sendServoCommand(servo, angle);
    });
  }

  Widget _buildLogoHeader() {
    return Container(
      height: 100,
      margin: const EdgeInsets.only(bottom: 12),
      child: Image.asset(
        'assets/logo.png',
        fit: BoxFit.contain,
      ),
    );
  }

  Widget _buildStatusHeader() {
    return Container(
      padding: const EdgeInsets.symmetric(vertical: 12, horizontal: 16),
      decoration: BoxDecoration(
        color: Colors.grey.shade100,
        borderRadius: BorderRadius.circular(16),
        boxShadow: const [
          BoxShadow(
            color: Color.fromARGB(255, 255, 255, 255),
            blurRadius: 6,
            offset: Offset(0, 2),
          ),
        ],
      ),
      margin: const EdgeInsets.only(bottom: 12),
      child: Row(
        children: [
          Icon(
            Icons.wifi,
            color: isConnected ? Colors.green : Colors.red,
            size: 28,
          ),
          const SizedBox(width: 12),
          Expanded(
            child: Text(
              isConnected ? "Connected to robot" : "Disconnected",
              style: TextStyle(
                fontSize: 16,
                fontWeight: FontWeight.w600,
                color: isConnected ? Colors.green : Colors.red,
              ),
            ),
          ),
          const Icon(Icons.battery_full, color: Color.fromARGB(255, 5, 233, 5)),
          const SizedBox(width: 6),
          Text(
            batteryVoltage != null ? "${batteryVoltage!.toStringAsFixed(1)} V" : "-- V",
            style: const TextStyle(fontWeight: FontWeight.w600),
          ),
        ],
      ),
    );
  }

  Widget _buildServoControl(int servo) {
    double min = 0;
    double max = 180;

    switch (servo) {
      case 1:
        max = 45;
        break;
      case 2:
        max = 130;
        break;
      case 3:
        max = 90;
        break;
    }

    final value = servoValues[servo]!.clamp(min, max);

    return Card(
      elevation: 3,
      color: Colors.white,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
      margin: const EdgeInsets.all(8),
      child: Padding(
        padding: const EdgeInsets.all(12),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Text(
              _getServoLabel(servo) + ' — ${value.round()}°',
              style: const TextStyle(fontWeight: FontWeight.bold),
            ),
            const SizedBox(height: 10),
            _buildVisualSlider(servo, value, min, max),
          ],
        ),
      ),
    );
  }

  Widget _buildVisualSlider(int servo, double value, double min, double max) {
    switch (servo) {
      case 1: // Gripper
        double offset = (45 - value);
        return Column(
          children: [
            SizedBox(
              height: 50,
              child: Stack(
                alignment: Alignment.center,
                children: [
                  Transform.translate(
                    offset: Offset(-offset, 0),
                    child: Container(width: 20, height: 40, color: Colors.blue),
                  ),
                  Transform.translate(
                    offset: Offset(offset, 0),
                    child: Container(width: 20, height: 40, color: Colors.blue),
                  ),
                ],
              ),
            ),
            Slider(
              value: value,
              min: min,
              max: max,
              divisions: (max - min).round(),
              label: '${value.round()}°',
              onChanged: (v) {
                setState(() {
                  servoValues[servo] = v;
                });
                _sendServoCommand(servo, v);
              },
            ),
          ],
        );

      case 2: // Height
        return Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            const Icon(Icons.arrow_upward, size: 24, color: Colors.blue),
            const SizedBox(height: 8),
            SizedBox(
              height: 90,
              child: RotatedBox(
                quarterTurns: -1,
                child: Slider(
                  value: value,
                  min: min,
                  max: max,
                  divisions: (max - min).round(),
                  label: '${value.round()}°',
                  activeColor: Colors.blue,
                  inactiveColor: Colors.blueGrey,
                  onChanged: (v) {
                    setState(() {
                      servoValues[servo] = v;
                    });
                    _sendServoCommand(servo, v);
                  },
                ),
              ),
            ),
            const SizedBox(height: 8),
            const Icon(Icons.arrow_downward, size: 24, color: Colors.blue),
          ],
        );

      case 3: // Reach
        return Column(
          children: [
            const Icon(Icons.unfold_more, size: 24, color: Colors.teal),
            Slider(
              value: value,
              min: min,
              max: max,
              divisions: (max - min).round(),
              label: '${value.round()}°',
              onChanged: (v) {
                setState(() {
                  servoValues[servo] = v;
                });
                _sendServoCommand(servo, v);
              },
            ),
          ],
        );

      case 4: // Base knob
        return Column(
          children: [
            CustomPaint(
              painter: KnobPainter(angle: value),
              size: const Size(100, 100),
            ),
            Slider(
              value: value,
              min: min,
              max: max,
              divisions: (max - min).round(),
              label: '${value.round()}°',
              onChanged: (v) {
                setState(() {
                  servoValues[servo] = v;
                });
                _sendServoCommand(servo, v);
              },
            ),
          ],
        );

      default:
        return Slider(
          value: value,
          min: min,
          max: max,
          divisions: (max - min).round(),
          label: '${value.round()}°',
          onChanged: (v) {
            setState(() {
              servoValues[servo] = v;
            });
            _sendServoCommand(servo, v);
          },
        );
    }
  }

  String _getServoLabel(int servo) {
    switch (servo) {
      case 1:
        return "Gripper";
      case 2:
        return "Height";
      case 3:
        return "Reach";
      case 4:
        return "Base";
      default:
        return "Servo $servo";
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.white,
      appBar: AppBar(
        title: const Text('Manual Control'),
        centerTitle: true,
        backgroundColor: Colors.blue,
        foregroundColor: Colors.white,
      ),
      body: SafeArea(
        child: Column(
          children: [
            _buildLogoHeader(),
            _buildStatusHeader(),
            Expanded(
              child: GridView.count(
                crossAxisCount: 2,
                childAspectRatio: 0.75,
                children: List.generate(4, (i) => _buildServoControl(i + 1)),
              ),
            ),
            Padding(
              padding: const EdgeInsets.symmetric(horizontal: 16.0, vertical: 12),
              child: ElevatedButton.icon(
                onPressed: _resetAllServos,
                icon: const Icon(Icons.restart_alt),
                label: const Text("Return to initial position"),
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.blue,
                  foregroundColor: Colors.white,
                  padding: const EdgeInsets.symmetric(vertical: 14, horizontal: 24),
                  textStyle: const TextStyle(fontSize: 16),
                  shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class KnobPainter extends CustomPainter {
  final double angle;
  KnobPainter({required this.angle});

  @override
  void paint(Canvas canvas, Size size) {
    final center = Offset(size.width / 2, size.height / 2);
    final radius = size.width / 2;

    final track = Paint()
      ..color = Colors.grey.shade300
      ..strokeWidth = 8
      ..style = PaintingStyle.stroke;

    final progress = Paint()
      ..color = Colors.blue
      ..strokeWidth = 10
      ..style = PaintingStyle.stroke
      ..strokeCap = StrokeCap.round;

    canvas.drawArc(
      Rect.fromCircle(center: center, radius: radius),
      pi,
      pi,
      false,
      track,
    );

    final sweepAngle = (angle / 180) * pi;
    canvas.drawArc(
      Rect.fromCircle(center: center, radius: radius),
      pi,
      sweepAngle,
      false,
      progress,
    );
  }

  @override
  bool shouldRepaint(covariant KnobPainter oldDelegate) {
    return oldDelegate.angle != angle;
  }
}
