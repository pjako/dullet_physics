import 'dart:io';
import 'package:analyzer_experimental/formatter.dart';

const options = const FormatterOptions();

CodeFormatter formater = new CodeFormatter(options);

void main() {
  List<FileSystemEntity> l = Directory.current.parent.listSync();
  for(FileSystemEntity folder in l) {
    var path = folder.path;
    if(path.endsWith('/lib')) {
      format(path);
    }
  }
  /*
  List<FileSystemEntity> l = Directory.current.listSync();
  for(FileSystemEntity fileDir in l) {
    var path = fileDir.path;
    var stats = fileDir.statSync();
    var type = fileDir.statSync().type;
    switch(type) {
      case(FileSystemEntityType.DIRECTORY):
        if(path.endsWith('packages')) continue;
        print('Directory: ${fileDir.path}');
        break;
      case(FileSystemEntityType.FILE):
        if(!path.endsWith('.dart')) continue;
        print('File: ${fileDir.path}');
        var file = new File(path);
        String source = file.readAsStringSync();
        FormattedSource formated = formater.format(CodeKind.COMPILATION_UNIT, source);
        print(formated.source);

        //var open = file.openSync(mode: FileMode.WRITE);
        break;
      default:
        break;
    }

  }*/

}

void format(String directory) {
  List<FileSystemEntity> l = new Directory(directory).listSync();
  for(FileSystemEntity fileDir in l) {
    var path = fileDir.path;
    var stats = fileDir.statSync();
    var type = fileDir.statSync().type;
    switch(type) {
      case(FileSystemEntityType.DIRECTORY):
        if(path.endsWith('packages')) continue;
        print('Directory: ${fileDir.path}');
        format(path);
        break;
      case(FileSystemEntityType.FILE):
        if(!path.endsWith('.dart')) continue;
        print('File: ${fileDir.path}');
        var file = new File(path);
        String source = file.readAsStringSync();
        FormattedSource formated = formater.format(CodeKind.COMPILATION_UNIT, source);
        //print(formated.source);

        var open = file.openSync(mode: FileMode.WRITE);
        open.writeStringSync(formated.source);
        open.closeSync();
        break;
      default:
        break;
    }

  }
}


/*
 *   List<FileSystemEntity> listSync({bool recursive: false,
                                   bool followLinks: true});
 */