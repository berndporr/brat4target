#include "window.h"

Window::Window()
{
	vLayout = new QVBoxLayout();

	lidarPlot = new QLIDARPlot;
	lidarPlot->addGraph();
	lidarPlot->graph(0)->setLineStyle(QCPGraph::lsNone);
	lidarPlot->graph(0)->setScatterStyle(QCPScatterStyle::ssDisc);
	
	lidarPlot->xAxis->setRange(-5, 5);
	lidarPlot->yAxis->setRange(-5, 5);

	vLayout->addWidget(lidarPlot);

	setLayout(vLayout);

	fprintf(stderr, "Starting screen update timer.\n");
	startTimer(std::chrono::milliseconds{100});
}

void Window::timerEvent(QTimerEvent *)
{
    const double w = sin(t);
    const double h = cos(t);
    QVector<double> x, y;
    for (int i = 0; i < ((rand()%100)+100); i++)
    {
	const double xp = ((double)rand() * 10 / RAND_MAX - 5)*w;
	const double yp = ((double)rand() * 10 / RAND_MAX - 5)*h;
	x.append(xp);
	y.append(yp);
    }
    lidarPlot->graph(0)->setData(x, y);
    t = t + 0.1;
    lidarPlot->replot();
}
