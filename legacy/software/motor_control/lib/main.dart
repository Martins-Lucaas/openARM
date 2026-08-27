import 'package:flutter/material.dart';
import 'package:web_socket_channel/web_socket_channel.dart';
import 'pages/manual_control_page.dart'; // Certifique-se de que esse caminho está correto

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    // Substitua pelo endereço correto do seu ESP32 ou servidor WebSocket
    final channel = WebSocketChannel.connect(
      Uri.parse('ws://192.168.4.1:80/ws'),
    );

    return MaterialApp(
      title: 'Controle do Braço Robótico',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        primarySwatch: Colors.blue,
      ),
      home: ManualControlPage(channel: channel),
    );
  }
}
