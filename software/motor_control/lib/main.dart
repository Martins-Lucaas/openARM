import 'package:flutter/material.dart';
import 'package:camera/camera.dart';
import 'pages/camera_control_page.dart';

// Variável global
late List<CameraDescription> cameras;

void main() async {
  WidgetsFlutterBinding.ensureInitialized();

  // Inicializa as câmeras
  cameras = await availableCameras();

  runApp(MyApp());
}

class MyApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Controle Robótico',
      theme: ThemeData(
        primarySwatch: Colors.blue,
      ),
      home: CameraControlPage(cameras: cameras), // passa para a tela
    );
  }
}
