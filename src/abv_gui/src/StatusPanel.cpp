#include "abv_gui/StatusPanel.h"
#include <QVBoxLayout>
#include <QFormLayout>
#include <QFrame>

static const QString kFieldStyle = "color: #888888; font-size: 11px;";
static const QString kValueStyle =
    "color: #ffffff;"
    "background-color: #2a2a2a;"
    "border: 1px solid #444444;"
    "border-radius: 4px;"
    "padding: 1px 6px;"
    "font-family: monospace;"
    "font-size: 13px;";
    
static const QString kHeaderStyle =
    "color: #cccccc;"
    "font-weight: bold;"
    "font-size: 14px;";

// ── Helpers ───────────────────────────────────────────────────────────────────

QLabel* StatusPanel::makeValueLabel()
{
    auto* label = new QLabel("---");
    label->setStyleSheet(kValueStyle);
    label->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    return label;
}

QWidget* StatusPanel::makeSectionHeader(const QString& title)
{
    auto* w      = new QWidget();
    auto* layout = new QVBoxLayout(w);
    layout->setContentsMargins(0, 4, 0, 2);
    layout->setSpacing(2);

    auto* label = new QLabel(title);
    label->setStyleSheet(kHeaderStyle);

    auto* line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setStyleSheet("color: #444444;");

    layout->addWidget(label);
    layout->addWidget(line);
    return w;
}

// ── Constructor ───────────────────────────────────────────────────────────────

StatusPanel::StatusPanel(QWidget* parent)
    : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(4, 4, 4, 4);
    layout->setSpacing(4);

    // ── Panel header ──────────────────────────────────────────────────────────
    auto* panelHeader = new QLabel("Status");
    panelHeader->setStyleSheet("color: #ffffff; font-weight: bold;");
    layout->addWidget(panelHeader);

    auto* topLine = new QFrame();
    topLine->setFrameShape(QFrame::HLine);
    topLine->setFrameShadow(QFrame::Sunken);
    layout->addWidget(topLine);

    // ── Guidance section ──────────────────────────────────────────────────────
    layout->addWidget(makeSectionHeader("Guidance"));

    auto* guidanceForm = new QFormLayout();
    guidanceForm->setSpacing(4);
    guidanceForm->setContentsMargins(4, 0, 0, 0);

    mGuidanceSmState = makeValueLabel();

    auto* guidanceSmLabel = new QLabel("SM State:");
    guidanceSmLabel->setStyleSheet(kFieldStyle);
    guidanceForm->addRow(guidanceSmLabel, mGuidanceSmState);
    layout->addLayout(guidanceForm);

    // ── Controller section ────────────────────────────────────────────────────
    layout->addWidget(makeSectionHeader("Controller"));

    auto* ctrlForm = new QFormLayout();
    ctrlForm->setSpacing(4);
    ctrlForm->setContentsMargins(4, 0, 0, 0);

    mControllerSmState   = makeValueLabel();
    mControllerCtrlState = makeValueLabel();

    auto* ctrlSmLabel   = new QLabel("SM State:");
    auto* ctrlCtrlLabel = new QLabel("Ctrl State:");
    ctrlSmLabel->setStyleSheet(kFieldStyle);
    ctrlCtrlLabel->setStyleSheet(kFieldStyle);

    ctrlForm->addRow(ctrlSmLabel,   mControllerSmState);
    ctrlForm->addRow(ctrlCtrlLabel, mControllerCtrlState);
    layout->addLayout(ctrlForm);

    layout->addStretch();
}

// ── Public API ────────────────────────────────────────────────────────────────

void StatusPanel::setGuidanceSmState(const QString& state)
{
    mGuidanceSmState->setText(state);
}

void StatusPanel::setControllerSmState(const QString& state)
{
    mControllerSmState->setText(state);
}

void StatusPanel::setControllerCtrlState(const QString& state)
{
    mControllerCtrlState->setText(state);
}