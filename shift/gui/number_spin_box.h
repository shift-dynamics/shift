#ifndef SHIFT_GUI_NUMBER_SPIN_BOX_H_
#define SHIFT_GUI_NUMBER_SPIN_BOX_H_

#include <QDoubleSpinBox>
#include <QKeyEvent>
#include <QRegularExpression>
#include <QSpinBox>

#include <cfloat>
#include <climits>
#include <cmath>
#include <functional>

namespace shift {
namespace gui {

/*!
@brief a spin box for a real number of any size, as a parameter of a model
takes: typed as a floating point number only (1.5, -2e-3, 4E+06), shown with
up to 12 significant digits, and stepped by its arrows by a tenth of the place
of its leading digit (9.81 by 0.1, 2.0e6 by 1.0e5, 0 by 0.1)
*/
class NumberSpinBox : public QDoubleSpinBox {
public:
  NumberSpinBox() {
    setRange(-DBL_MAX, DBL_MAX);
    setDecimals(DBL_MAX_10_EXP + DBL_DIG); // no rounding of the value typed
    setKeyboardTracking(false);
    setCorrectionMode(QAbstractSpinBox::CorrectToPreviousValue);
  }

  /*! called when Enter is pressed in the box */
  std::function<void()> on_return;

  QString textFromValue(double value) const override {
    return QString::number(value, 'g', 12);
  }

  double valueFromText(const QString& text) const override {
    return text.trimmed().toDouble();
  }

  QValidator::State validate(QString& text, int&) const override {
    const QString trimmed = text.trimmed();
    bool ok = false;
    const double value = trimmed.toDouble(&ok);
    if(ok && std::isfinite(value)) {
      return QValidator::Acceptable;
    }
    // on the way to a number: "", "-", "1.", "2e", "2e-"
    static const QRegularExpression partial(
      "^[+-]?[0-9]*\\.?[0-9]*([eE][+-]?[0-9]*)?$");
    return partial.match(trimmed).hasMatch() ? QValidator::Intermediate
                                             : QValidator::Invalid;
  }

  void fixup(QString& text) const override {
    text = textFromValue(value());
  }

  void stepBy(int steps) override {
    const double magnitude = std::fabs(value());
    const double step =
      magnitude > 0.0 ? std::pow(10.0, std::floor(std::log10(magnitude)) - 1.0)
                      : 0.1;
    setValue(value() + steps * step);
  }

protected:
  void keyPressEvent(QKeyEvent* event) override {
    QDoubleSpinBox::keyPressEvent(event);
    if((event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter) &&
       on_return) {
      on_return();
    }
  }
};

/*! @brief a spin box for an integer of any size, typed as digits only */
class IntegerSpinBox : public QSpinBox {
public:
  IntegerSpinBox() {
    setRange(INT_MIN, INT_MAX);
    setKeyboardTracking(false);
    setCorrectionMode(QAbstractSpinBox::CorrectToPreviousValue);
  }

  /*! called when Enter is pressed in the box */
  std::function<void()> on_return;

protected:
  void keyPressEvent(QKeyEvent* event) override {
    QSpinBox::keyPressEvent(event);
    if((event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter) &&
       on_return) {
      on_return();
    }
  }
};

} // namespace gui
} // namespace shift

#endif
