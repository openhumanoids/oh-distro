
#include <QScriptEngine>
#include <QScriptValueIterator>
#include <QFile>
#include <QDebug>

#include <iostream>

class Json {

public:

  Json()
  {
  }


  static QString encode(const QMap<QString,QVariant> &map)
  {
      QScriptEngine engine;
      engine.evaluate("function toString() { return JSON.stringify(this) }");

      QScriptValue toString = engine.globalObject().property("toString");
      QScriptValue obj = encodeInner(map, &engine);
      return toString.call(obj).toString();

  }

  static QMap<QString, QVariant> decode(const QString &jsonStr)
  {
      QScriptValue object;
      QScriptEngine engine;
      object = engine.evaluate("(" + jsonStr + ")");
      return decodeInner(object);
  }

  static QMap<QString, QVariant> decodeFile(const QString &filename)
  {
    QString fileContents;
    QFile file(filename);
    if (file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
      fileContents = file.readAll();
    }
    else
    {
      qDebug() << "Json::decodeFile: Failed to open file: " << filename;
    }

    return Json::decode(fileContents);
  }

  static void encodeFile(const QString &filename, const QMap<QString,QVariant> &map)
  {
    QFile file(filename);
    if (!file.open(QIODevice::ReadWrite | QIODevice::Text))
    {
      qDebug() << "Json::encodeFile: Failed to open file: " << filename;
      return;
    }

    QTextStream stream(&file);
    stream << Json::encode(map) << endl;

  }

  protected:

  static QScriptValue encodeInner(const QMap<QString,QVariant> &map, QScriptEngine* engine)
  {
      QScriptValue obj = engine->newObject();
      QMapIterator<QString, QVariant> i(map);
      while (i.hasNext()) {
          i.next();
          if (i.value().type() == QVariant::String)
              obj.setProperty(i.key(), i.value().toString());
          else if (i.value().type() == QVariant::Int)
              obj.setProperty(i.key(), i.value().toInt());
          else if (i.value().type() == QVariant::Double)
              obj.setProperty(i.key(), i.value().toDouble());
          else if (i.value().type() == QVariant::List)
              obj.setProperty(i.key(), qScriptValueFromSequence(engine, i.value().toList()));
          else if (i.value().type() == QVariant::Map)
              obj.setProperty(i.key(), encodeInner(i.value().toMap(), engine));
      }
      return obj;
  }


  static QMap<QString, QVariant> decodeInner(QScriptValue object)
  {
      QMap<QString, QVariant> map;
      QScriptValueIterator it(object);
      while (it.hasNext()) {
          it.next();
          if (it.value().isArray())
              map.insert(it.name(),QVariant(decodeInnerToList(it.value())));
          else if (it.value().isNumber())
              map.insert(it.name(),QVariant(it.value().toNumber()));
          else if (it.value().isString())
              map.insert(it.name(),QVariant(it.value().toString()));
          else if (it.value().isNull())
              map.insert(it.name(),QVariant());
          else if(it.value().isObject())
              map.insert(it.name(),QVariant(decodeInner(it.value())));
      }
      return map;
  }

  static QList<QVariant> decodeInnerToList(QScriptValue arrayValue)
  {
      QList<QVariant> list;
      QScriptValueIterator it(arrayValue);
      while (it.hasNext()) {
          it.next();
          if (it.name() == "length")
              continue;

          if (it.value().isArray())
              list.append(QVariant(decodeInnerToList(it.value())));
          else if (it.value().isNumber())
              list.append(QVariant(it.value().toNumber()));
          else if (it.value().isString())
              list.append(QVariant(it.value().toString()));
          else if (it.value().isNull())
              list.append(QVariant());
          else if(it.value().isObject())
              list.append(QVariant(decodeInner(it.value())));
      }
      return list;
  }

};
