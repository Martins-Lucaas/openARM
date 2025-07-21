import 'dart:convert';
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
    1: 90,
    2: 90,
    3: 90,
    4: 90,
  };

  void _sendServoCommand(int servo, double angle) {
    final command = {
      'servo': servo,
      'angulo': angle.round().clamp(0, 180),
    };
    try {
      widget.channel.sink.add(jsonEncode(command));
    } catch (e) {
      print('Erro ao enviar comando manual: $e');
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Controle Manual'),
      ),
      body: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          children: List.generate(4, (i) {
            final servo = i + 1;
            return Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text('Servo $servo (0–180°): ${servoValues[servo]!.round()}'),
                Slider(
                  value: servoValues[servo]!,
                  min: 0,
                  max: 180,
                  divisions: 180,
                  label: servoValues[servo]!.round().toString(),
                  onChanged: (value) {
                    setState(() {
                      servoValues[servo] = value;
                    });
                    _sendServoCommand(servo, value);
                  },
                ),
                const SizedBox(height: 12),
              ],
            );
          }),
        ),
      ),
    );
  }
}
