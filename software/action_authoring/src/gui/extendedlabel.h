#ifndef EXTENDEDQLABEL_H
#define EXTENDEDQLABEL_H

#include <QLabel>

class ExtendedQLabel : public QLabel
{
    Q_OBJECT

public:
    explicit ExtendedQLabel(const QString &text ="", QWidget *parent = 0);
    ~ExtendedQLabel();

signals:
    void clicked();
    void hover();

protected:
    void mouseMoveEvent(QMouseEvent *e);
    void mouseReleaseEvent(QMouseEvent *e);
};

#endif /* EXTENDEDQLABEL_H */
