#ifndef BUTTONADAPTER_HPP
#define BUTTONADAPTER_HPP

#include <QWidget>
#include <QPushButton>
#include <QString>
#include <QLayout> 

struct ButtonStyle {
    QString styleSheet;
    QSize fixedSize;

        static QString makeStyleSheet(const QString& base, const QString& hover, const QString& pressed) {
        return QString(R"(
            QPushButton {
                background-color: %1;
                color: white;
                border: none;
                border-radius: 6px;
                padding: 6px 16px;
                font-size: 13px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: %2; }
            QPushButton:pressed { background-color: %3; padding-top: 7px; padding-bottom: 5px; }
            QPushButton:disabled { background-color: #7f8c8d; color: #bdc3c7; }
        )").arg(base, hover, pressed);
    }

    static ButtonStyle primary() { return { makeStyleSheet("#2980b9", "#3498db", "#1a6fa0"), {} }; }
    static ButtonStyle danger()  { return { makeStyleSheet("#c0392b", "#e74c3c", "#a93226"), {} }; }
    static ButtonStyle warning() { return { makeStyleSheet("#d68910", "#f39c12", "#b7770d"), {} }; }

};

class ButtonAdapter : public QWidget {
    Q_OBJECT
public:
    using Action = std::function<void()>;

    ButtonAdapter(const std::string& name,
                  Action action,
                  ButtonStyle style = {},
                  QWidget* parent = nullptr)
    : QWidget(parent), mAction(std::move(action))
    {
        mButton = new QPushButton(QString::fromStdString(name), this);
        
        mButton->setStyleSheet(style.styleSheet);
        if (!style.fixedSize.isEmpty())
            mButton->setFixedSize(style.fixedSize);

        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->addWidget(mButton);
        
        connect(mButton, &QPushButton::clicked, [this]() { mAction(); });
    }

    QPushButton* button() {return mButton;}

    void resize(int minw, int minh) 
    {
        mButton->setMinimumSize(minw, minh); 
        QFont f = mButton->font();
        f.setPointSize(mButton->height() / 2);  // ratio you can tune
        mButton->setFont(f);
    }

private:
    Action mAction;
    QPushButton* mButton;
};
#endif // BUTTONADAPTER_HPP