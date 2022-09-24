import numpy as np
from scipy.special import erf


def kde_pdf(data, kernel_func, bandwidth):
    """Generate kernel density estimator over data."""
    kernels = dict()
    n = len(data)
    for d in data:
        kernels[d] = kernel_func(d, bandwidth)

    def evaluate(x):
        """Evaluate `x` using kernels above."""
        pdfs = list()
        for dd in data:
            pdfs.append(kernels[dd](x))

        return sum(pdfs) / n

    return evaluate


def kde_cdf(data, kernel_func, bandwidth):
    """Generate kernel distribution estimator over data."""
    kernels = dict()
    n = len(data)
    for d in data:
        kernels[d] = kernel_func(d, bandwidth)

    def evaluate(x):
        """Evaluate x using kernels above."""
        cdfs = list()
        for dd in data:
            cdfs.append(kernels[dd](x))

        return sum(cdfs) / n
    return evaluate


def gaussian_pdf(xi, bandwidth):
    """ Return Gaussian kernel density estimator.

    INPUTS
    ------
    xi: something
    bandwidth: something

    OUTPUTS
    -------
    evaluate: function

    """
    x_bar = xi

    def evaluate(x):
        """ Evaluate x."""
        return (np.sqrt(2*np.pi*bandwidth**2)**-1) * np.exp(-((x - x_bar)**2)/(2*bandwidth**2))

    return evaluate


def gaussian_cdf(xi, bandwidth):
    """ Return Gaussian kernel density estimator.

    INPUTS
    ------
    xi: something
    bandwidth: something

    OUTPUTS
    -------
    evaluate: function

    """
    x_bar = xi

    def evaluate(x):
        """ Evaluate x."""
        return -1 / 2 * erf((x_bar - x) / (bandwidth * np.sqrt(2)))

    return evaluate


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    import seaborn as sns
    from random import gauss

    sns.set(color_codes=True)
    plt.rcParams["figure.figsize"] = (15, 10)

    # Build example dataset
    mu = 1.0
    sigma = 3.0
    vals = []
    n = 100
    for ii in range(n):
        xx = gauss(mu, sigma)
        yy = gauss(0, sigma/2)
        vals.append(xx**2 + yy**2)
    silvermans = (4 * sigma**5 / (3*n))**(1/5)
    print("Silvermans: {:.2f}".format(silvermans))
    # vals = [5, 12, 15, 20]
    xvals = np.arange(min(vals), max(vals), .01)

    fig = plt.figure()

    # bandwidth=optimal (Silverman's rule of thumb):
    ax1 = fig.add_subplot(2, 2, 1)
    dist_1 = kde_pdf(data=vals, kernel_func=gaussian_pdf, bandwidth=silvermans)
    y1 = [dist_1(i) for i in xvals]
    ys1 = [dist_1(i) for i in vals]
    ax1.scatter(vals, ys1)
    ax1.plot(xvals, y1)

    # bandwidth=2:
    ax2 = fig.add_subplot(2, 2, 2)
    dist_2 = kde_pdf(data=vals, kernel_func=gaussian_pdf, bandwidth=2)
    y2 = [dist_2(i) for i in xvals]
    ys2 = [dist_2(i) for i in vals]
    ax2.scatter(vals, ys2)
    ax2.plot(xvals, y2)

    # bandwidth=3:
    ax3 = fig.add_subplot(2, 2, 3)
    dist_3 = kde_pdf(vals, kernel_func=gaussian_pdf, bandwidth=3)
    y3 = [dist_3(i) for i in xvals]
    ys3 = [dist_3(i) for i in vals]
    ax3.scatter(vals, ys3)
    ax3.plot(xvals, y3)

    # bandwidth=4:
    ax4 = fig.add_subplot(2, 2, 4)
    dist_4 = kde_pdf(vals, kernel_func=gaussian_pdf, bandwidth=4)
    y4 = [dist_4(i) for i in xvals]
    ys4 = [dist_4(i) for i in vals]
    ax4.scatter(vals, ys4)
    ax4.plot(xvals, y4)

    # display gridlines
    g1 = ax1.grid(True)
    g2 = ax2.grid(True)
    g3 = ax3.grid(True)
    g4 = ax4.grid(True)

    # set title
    t1 = ax1.set_title(r"Gaussian Kernel")
    t2 = ax2.set_title(r"Gaussian Kernel")
    t3 = ax3.set_title(r"Gaussian Kernel")
    t4 = ax4.set_title(r"Gaussian Kernel")

    # display legend in each subplot
    leg1 = mpatches.Patch(color=None, label='bandwidth=1')
    leg2 = mpatches.Patch(color=None, label='bandwidth=2')
    leg3 = mpatches.Patch(color=None, label='bandwidth=3')
    leg4 = mpatches.Patch(color=None, label='bandwidth=4')

    ax1.legend(handles=[leg1])
    ax2.legend(handles=[leg2])
    ax3.legend(handles=[leg3])
    ax4.legend(handles=[leg4])

    plt.tight_layout()
    plt.show()