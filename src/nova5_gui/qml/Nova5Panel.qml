import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

GroupBox {
  title: "Nova5 Control"
  Layout.fillWidth: true

  ColumnLayout {
    spacing: 8
    anchors.margins: 8

    // Enable
    RowLayout {
      Label { text: "Load:" }

      // SpinBox int + nhập từ bàn phím (x10 để có 0.1)
      SpinBox {
        id: loadBox              // 0.0..10.0, step 0.1
        from: 0;  to: 100; stepSize: 1
        value: 35                 // 3.5 kg mặc định -> 35
        editable: true
        // Hiển thị kèm đơn vị
        textFromValue: function(v, locale) { return (v/10).toFixed(1) + " kg"; }
        // Cho phép gõ "3.5" hoặc "3.5 kg"
        valueFromText: function(t, locale) {
          var n = parseFloat(t); if (isNaN(n)) n = 0; return Math.round(n*10);
        }
        // helper để lấy double:
        property real kg: value/10.0
        Layout.preferredWidth: 120
      }

      Button { text: "Enable"; onClicked: nova5.enableRobot(loadBox.kg) }
    }

    // GetAngle + Save
    RowLayout {
      Button { text: "GetAngle"; onClicked: nova5.getAngles() }
      //Button { text: "Save YAML"; onClicked: nova5.saveAnglesYaml("/home/pi/waypoints.yaml") }
      Button {
        text: "Save YAML"
        onClicked: {
          function d(i) { return sbRepeater.itemAt(i).deg }
          var arr = [ d(0), d(1), d(2), d(3), d(4), d(5) ]
          nova5.saveAnglesYaml("/home/pi/waypoints.yaml", arr)
        }
      }
    }

    // === Delegate chuẩn cho 1 ô góc ===
    Component {
      id: jointSpinDelegate
      SpinBox {
        from: -3600; to: 3600; stepSize: 1       // -360.0 .. 360.0 (x10)
        value: Math.round(((nova5.angles && nova5.angles.length > index) ? nova5.angles[index] : 0) * 10)
        editable: true
        textFromValue: function(v, locale) { return (v/10).toFixed(1) + "°"; }
        valueFromText: function(t, locale) {
          var n = parseFloat(t); if (isNaN(n)) n = 0; return Math.round(n*10);
        }
        // đọc ra double:
        property real deg: value / 10.0
        Layout.preferredWidth: 110
        // Enter để bỏ focus (commit)
        Keys.onReturnPressed: focus = false
        Keys.onEnterPressed:  focus = false
      }
    }

    // 6 ô J1..J6
    RowLayout {
      id: spinRow
      spacing: 6
      Repeater {
        id: sbRepeater
        model: 6
        delegate: jointSpinDelegate
      }
    }

    // Gửi JointMovJ
    RowLayout {
      spacing: 6
      Button {
        text: "Send JointMovJ"
        onClicked: {
          function d(i){ return sbRepeater.itemAt(i).deg }
          nova5.jointMovJ(d(0), d(1), d(2), d(3), d(4), d(5))
        }
      }
    }
  }
}
