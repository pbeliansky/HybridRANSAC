
import numpy as np
import matplotlib.pyplot as plt
import matplotx

# plt.rcParams.update({'font.size': 22})
plt.rcParams['font.size'] = 13
plt.rcParams['pdf.fonttype'] = 42

import random
import os


def load_scores(hybrid_path_and_name, random_path_and_name, hr, hrm, r, rm):
    # load Inlier mean
    path = hybrid_path_and_name + hr
    with open(path, "r") as f:
        lines = f.readlines()

    scores_hr = np.zeros(len(lines))
    for i, l in enumerate(lines):
        scores_hr[i] = float(l)


    # load Inlier median
    path = hybrid_path_and_name + hrm
    with open(path, "r") as f:
        lines = f.readlines()

    scores_hr_med = np.zeros(len(lines))
    for i, l in enumerate(lines):
        scores_hr_med[i] = float(l)



    # load ransac mean
    path = random_path_and_name + r
    with open(path, "r") as f:
        lines = f.readlines()

    scores_r = np.zeros(len(lines))
    for i, l in enumerate(lines):
        scores_r[i] = float(l)


    # load ransac median
    path = random_path_and_name + rm
    with open(path, "r") as f:
        lines = f.readlines()

    scores_r_med = np.zeros(len(lines))
    for i, l in enumerate(lines):
        scores_r_med[i] = float(l)

    return scores_hr, scores_hr_med, scores_r, scores_r_med



#*
def plot_scores(hybrid_path_and_name, random_path_and_name, fig_name, save_folder=None, mean=True, median=True, filename=None, fr=100,
                hr="_mean_scores_.txt",
                hrm="_median_scores_.txt",
                r="_mean_scores_.txt",
                rm="_median_scores_.txt",
                y_label='best model score', 
                x_label='#iterations'):
    if filename == None:
        filename = fig_name
    scores_hr, scores_hr_med, scores_r, scores_r_med = load_scores(hybrid_path_and_name, random_path_and_name, hr, hrm, r, rm)

    rfirst_zero = np.where(scores_r == scores_r[-1])[0][0]
    hrfirst_zero = np.where(scores_hr == scores_hr[-1])[0][0]
    mrfirst_zero = np.where(scores_r_med == scores_r_med[-1])[0][0]
    mhrfirst_zero = np.where(scores_hr_med == scores_hr_med[-1])[0][0]

    first_zero = max(rfirst_zero, hrfirst_zero, mrfirst_zero, mhrfirst_zero)
    x = np.arange(1, first_zero + 1)


    fig, ax = plt.subplots()
    if mean:
        ax.plot(x[fr:hrfirst_zero], scores_hr[fr  :   hrfirst_zero], label='hybrid_ransac_mean')
        ax.plot(x[fr:rfirst_zero], scores_r[fr  :   rfirst_zero], label='ransac_mean')
    if median:
        ax.plot(x[fr:mhrfirst_zero], scores_hr_med[fr  :   mhrfirst_zero], label='hybrid_ransac_median')
        ax.plot(x[fr:mrfirst_zero], scores_r_med[fr  :   mrfirst_zero], label='ransac_median')

    ax.set_xlabel(x_label)  # Add an x-label to the axes.
    ax.set_ylabel(y_label)  # Add a y-label to the axes.
    ax.set_title(fig_name)  # Add a title to the axes.
    ax.legend();  # Add a legend.
    # plt.show()
    if save_folder is not None:
        plt.savefig(save_folder + "/" + filename + ".png", bbox_inches='tight')






def plot_3_scores(hybrid_path_and_name, random_path_and_name, oracle_path_and_name, fig_name, save_folder=None, mean=True, median=True, filename=None, to=100,
                hr="_mean_scores_.txt",
                hrm="_median_scores_.txt",
                r="_mean_scores_.txt",
                rm="_median_scores_.txt",
                y_label='best model score', x_label='#iterations'):
    if filename == None:
        filename = fig_name
        
    scores_hr, scores_hr_med, scores_r, scores_r_med = load_scores(hybrid_path_and_name, random_path_and_name, hr, hrm, r, rm)
    scores_hro, scores_hro_med, _, _ = load_scores(oracle_path_and_name, oracle_path_and_name, hr, hrm, r, rm)

    rfirst_zero = to # np.where(scores_r == scores_r[-1])[0][0]
    hrfirst_zero = to # np.where(scores_hr == scores_hr[-1])[0][0]
    hrofirst_zero = to # np.where(scores_hro == scores_hro[-1])[0][0]
    mrfirst_zero = to # np.where(scores_r_med == scores_r_med[-1])[0][0]
    mhrfirst_zero = to # np.where(scores_hr_med == scores_hr_med[-1])[0][0]
    mhrofirst_zero = to # np.where(scores_hro_med == scores_hro_med[-1])[0][0]

    first_zero = to # max(rfirst_zero, hrfirst_zero, hrofirst_zero, mrfirst_zero, mhrfirst_zero, mhrofirst_zero)
    x = np.arange(1, first_zero + 1)

    fr = 0

    mean_label = ""
    median_label = ""

    if mean and median:
        mean_label = " Mean"
        median_label = " Median"

    fig, ax = plt.subplots()
    if mean:
        ax.plot(x[fr:rfirst_zero], scores_r[fr  :   rfirst_zero], label='Random' + mean_label)
        ax.plot(x[fr:hrfirst_zero], scores_hr[fr  :   hrfirst_zero], label='Inlier' + mean_label)
        ax.plot(x[fr:hrofirst_zero], scores_hro[fr  :   hrofirst_zero], label='Oracle' + mean_label)
    if median:
        ax.plot(x[fr:mrfirst_zero], scores_r_med[fr  :   mrfirst_zero], label='Random' + median_label)
        ax.plot(x[fr:mhrfirst_zero], scores_hr_med[fr  :   mhrfirst_zero], label='Inlier' + median_label)
        ax.plot(x[fr:mhrofirst_zero], scores_hro_med[fr  :   mhrofirst_zero], label='Oracle' + median_label)

    ax.set_xlabel(x_label)  # Add an x-label to the axes.
    ax.set_ylabel(y_label)  # Add a y-label to the axes.
    ax.set_title(fig_name)  # Add a title to the axes.
    ax.legend();  # Add a legend.
    # plt.show()
    if save_folder is not None:
        plt.savefig(save_folder + "/" + filename + ".png", bbox_inches='tight')

    # for overleaf

    pdf_save_folder = save_folder[:save_folder.find("results_and_plots")] + save_folder[save_folder.find("results_and_plots") + 12:]
    overleaf_load_file = "img/" + save_folder[save_folder.find("results_and_plots") + 12:] + "/" + filename.replace(" ", "_") + ".pdf"

    os.makedirs(pdf_save_folder, exist_ok=True) 
    plt.savefig(pdf_save_folder + "/" + filename.replace(" ", "_") + ".pdf", format="pdf", bbox_inches="tight")

    subplot_string = r"""\begin{subfigure}[b]{\textwidth}
    \centering
    \includegraphics[width=\textwidth]{""" + overleaf_load_file + r"""}
    \caption{""" + y_label + r"""}
    \label{fig:""" + fig_name.replace(" ", "_") + "_" + filename.replace(" ", "_") + r"""}
\end{subfigure}"""

    with open(pdf_save_folder + "/" + filename.replace(" ", "_") + ".txt", "w") as text_file:
        text_file.write(subplot_string)

    with open(pdf_save_folder + "/../../../../overleaf_accumulator.txt", "a+") as text_file:
        text_file.write(subplot_string)
        text_file.write("\n\n\n")





def plot_3_dist_scores(hybrid_path_and_name, random_path_and_name, oracle_path_and_name, fig_name, N=9, save_folder=None, mean=True, median=True, filename=None, to=100,
                hr="_mean_scores_.txt",
                hrm="_median_scores_.txt",
                r="_mean_scores_.txt",
                rm="_median_scores_.txt",
                y_label='best model score', x_label='#iterations'):
    if filename == None:
        filename = fig_name
        
    scores_hr = []
    scores_hr_med= []
    scores_r= []
    scores_r_med= []
    scores_hro= []
    scores_hro_med= []

    expr_range = range(N)
    
    for i in expr_range:
        if N == 9 and i not in [0, 4, 8]:
            scores_hr.append([])
            scores_hr_med.append([])
            scores_r.append([])
            scores_r_med.append([])
            scores_hro.append([])
            scores_hro_med.append([])
        else:
            s_hr, s_hr_med, s_r, s_r_med = load_scores(hybrid_path_and_name + str(i) + "/res", random_path_and_name + str(i) + "/res", hr, hrm, r, rm)
            s_hro, s_hro_med, _, _ = load_scores(oracle_path_and_name + str(i) + "/res", oracle_path_and_name + str(i) + "/res", hr, hrm, r, rm)

            scores_hr.append(s_hr)
            scores_hr_med.append(s_hr_med)
            scores_r.append(s_r)
            scores_r_med.append(s_r_med)
            scores_hro.append(s_hro)
            scores_hro_med.append(s_hro_med)

    
    if N == 9:
        expr_range = [0, 4, 8]

    rfirst_zero = to # np.where(scores_r == scores_r[-1])[0][0]
    hrfirst_zero = to # np.where(scores_hr == scores_hr[-1])[0][0]
    hrofirst_zero = to # np.where(scores_hro == scores_hro[-1])[0][0]
    mrfirst_zero = to # np.where(scores_r_med == scores_r_med[-1])[0][0]
    mhrfirst_zero = to # np.where(scores_hr_med == scores_hr_med[-1])[0][0]
    mhrofirst_zero = to # np.where(scores_hro_med == scores_hro_med[-1])[0][0]

    first_zero = to # max(rfirst_zero, hrfirst_zero, hrofirst_zero, mrfirst_zero, mhrfirst_zero, mhrofirst_zero)
    x = np.arange(1, first_zero + 1)

    fr = 0

    mean_label = ""
    median_label = ""

    if mean and median:
        mean_label = " Mean"
        median_label = " Median"


    fig, ax = plt.subplots()
    for i in expr_range:
        if N == 2:
            dist_type = " ascending" if i==0 else " descending" 
        elif N == 9:
            if len(expr_range) == 3:
                if i == 0:
                    dist_type = " closest"
                if i == 4:
                    dist_type = " middle"
                if i == 8:
                    dist_type = " farthest"
            else:
                dist_type = " dist" + str(i)
        else:
            dist_type = " dist" + str(i)

        if mean:
            ax.plot(x[fr:rfirst_zero], scores_r[i][fr  :   rfirst_zero], label='Random' + mean_label + dist_type)
            ax.plot(x[fr:hrfirst_zero], scores_hr[i][fr  :   hrfirst_zero], label='Inlier' + mean_label + dist_type)
            ax.plot(x[fr:hrofirst_zero], scores_hro[i][fr  :   hrofirst_zero], label='Oracle' + mean_label + dist_type)
        if median:
            ax.plot(x[fr:mrfirst_zero], scores_r_med[i][fr  :   mrfirst_zero], label='Random' + median_label + dist_type)
            ax.plot(x[fr:mhrfirst_zero], scores_hr_med[i][fr  :   mhrfirst_zero], label='Inlier' + median_label + dist_type)
            ax.plot(x[fr:mhrofirst_zero], scores_hro_med[i][fr  :   mhrofirst_zero], label='Oracle' + median_label + dist_type)

    ax.set_xlabel(x_label)  # Add an x-label to the axes.
    ax.set_ylabel(y_label)  # Add a y-label to the axes.
    ax.set_title(fig_name)  # Add a title to the axes.
    ax.legend();  # Add a legend.
    # plt.show()
    if save_folder is not None:
        plt.savefig(save_folder + "/" + filename + ".png", bbox_inches='tight')

    # for overleaf

    pdf_save_folder = save_folder[:save_folder.find("results_and_plots")] + save_folder[save_folder.find("results_and_plots") + 12:]
    overleaf_load_file = "img/" + save_folder[save_folder.find("results_and_plots") + 12:] + "/" + filename.replace(" ", "_") + ".pdf"

    os.makedirs(pdf_save_folder, exist_ok=True) 
    plt.savefig(pdf_save_folder + "/" + filename.replace(" ", "_") + ".pdf", format="pdf", bbox_inches="tight")

    subplot_string = r"""\begin{subfigure}[b]{\textwidth}
    \centering
    \includegraphics[width=\textwidth]{""" + overleaf_load_file + r"""}
    \caption{""" + y_label + r"""}
    \label{fig:""" + fig_name.replace(" ", "_") + "_" + filename.replace(" ", "_") + r"""}
\end{subfigure}"""

    with open(pdf_save_folder + "/" + filename.replace(" ", "_") + ".txt", "w") as text_file:
        text_file.write(subplot_string)

    with open(pdf_save_folder + "/../../../../overleaf_accumulator.txt", "a+") as text_file:
        text_file.write(subplot_string)
        text_file.write("\n\n\n")



    ###################################################################
    # only hybrid now

    fig, ax = plt.subplots()

    for i in expr_range:
        if N == 2:
            dist_type = " ascending" if i==0 else " descending" 
        elif N == 9:
            if len(expr_range) == 3:
                if i == 0:
                    dist_type = " closest"
                if i == 4:
                    dist_type = " middle"
                if i == 8:
                    dist_type = " farthest"
            else:
                dist_type = " dist" + str(i)
        else:
            dist_type = " dist" + str(i)

        if mean:
            ax.plot(x[fr:hrfirst_zero], scores_hr[i][fr  :   hrfirst_zero], label='Inlier' + mean_label + dist_type)
        if median:
            ax.plot(x[fr:mhrfirst_zero], scores_hr_med[i][fr  :   mhrfirst_zero], label='Inlier' + median_label + dist_type)

    ax.set_xlabel(x_label)  # Add an x-label to the axes.
    ax.set_ylabel(y_label)  # Add a y-label to the axes.
    ax.set_title(fig_name)  # Add a title to the axes.
    ax.legend();  # Add a legend.
    # plt.show()
    if save_folder is not None:
        plt.savefig(save_folder + "/" + filename + "_hybrid.png")

    # for overleaf

    pdf_save_folder = save_folder[:save_folder.find("results_and_plots")] + save_folder[save_folder.find("results_and_plots") + 12:]
    overleaf_load_file = "img/" + save_folder[save_folder.find("results_and_plots") + 12:] + "/" + filename.replace(" ", "_") + "_hybrid.pdf"

    os.makedirs(pdf_save_folder, exist_ok=True) 
    plt.savefig(pdf_save_folder + "/" + filename.replace(" ", "_") + "_hybrid.pdf", format="pdf", bbox_inches="tight")

    subplot_string = r"""\begin{subfigure}[b]{\textwidth}
    \centering
    \includegraphics[width=\textwidth]{""" + overleaf_load_file + r"""}
    \caption{""" + y_label + r"""}
    \label{fig:""" + fig_name.replace(" ", "_") + "_" + filename.replace(" ", "_") + r"""_hybrid}
\end{subfigure}"""

    with open(pdf_save_folder + "/" + filename.replace(" ", "_") + "_hybrid.txt", "w") as text_file:
        text_file.write(subplot_string)

    with open(pdf_save_folder + "/../../../../overleaf_accumulator.txt", "a+") as text_file:
        text_file.write(subplot_string)
        text_file.write("\n\n\n")




    ###################################################################
    # only ransac now

    fig, ax = plt.subplots()

    for i in expr_range:
        if N == 2:
            dist_type = " ascending" if i==0 else " descending" 
        elif N == 9:
            if len(expr_range) == 3:
                if i == 0:
                    dist_type = " closest"
                if i == 4:
                    dist_type = " middle"
                if i == 8:
                    dist_type = " farthest"
            else:
                dist_type = " dist" + str(i)
        else:
            dist_type = " dist" + str(i)

        if mean:
            ax.plot(x[fr:rfirst_zero], scores_r[i][fr  :   rfirst_zero], label='Random' + mean_label + dist_type)
        if median:
            ax.plot(x[fr:mrfirst_zero], scores_r_med[i][fr  :   mrfirst_zero], label='Random' + median_label + dist_type)

    ax.set_xlabel(x_label)  # Add an x-label to the axes.
    ax.set_ylabel(y_label)  # Add a y-label to the axes.
    ax.set_title(fig_name)  # Add a title to the axes.
    ax.legend();  # Add a legend.
    # plt.show()
    if save_folder is not None:
        plt.savefig(save_folder + "/" + filename + "_random.png")

    # for overleaf

    pdf_save_folder = save_folder[:save_folder.find("results_and_plots")] + save_folder[save_folder.find("results_and_plots") + 12:]
    overleaf_load_file = "img/" + save_folder[save_folder.find("results_and_plots") + 12:] + "/" + filename.replace(" ", "_") + "_random.pdf"

    os.makedirs(pdf_save_folder, exist_ok=True) 
    plt.savefig(pdf_save_folder + "/" + filename.replace(" ", "_") + "_random.pdf", format="pdf", bbox_inches="tight")

    subplot_string = r"""\begin{subfigure}[b]{\textwidth}
    \centering
    \includegraphics[width=\textwidth]{""" + overleaf_load_file + r"""}
    \caption{""" + y_label + r"""}
    \label{fig:""" + fig_name.replace(" ", "_") + "_" + filename.replace(" ", "_") + r"""_random}
\end{subfigure}"""

    with open(pdf_save_folder + "/" + filename.replace(" ", "_") + "_random.txt", "w") as text_file:
        text_file.write(subplot_string)

    with open(pdf_save_folder + "/../../../../overleaf_accumulator.txt", "a+") as text_file:
        text_file.write(subplot_string)
        text_file.write("\n\n\n")



    ###################################################################
    # only oracle now

    fig, ax = plt.subplots()

    for i in expr_range:
        if N == 2:
            dist_type = " ascending" if i==0 else " descending" 
        elif N == 9:
            if len(expr_range) == 3:
                if i == 0:
                    dist_type = " closest"
                if i == 4:
                    dist_type = " middle"
                if i == 8:
                    dist_type = " farthest"
            else:
                dist_type = " dist" + str(i)
        else:
            dist_type = " dist" + str(i)

        if mean:
            ax.plot(x[fr:hrofirst_zero], scores_hro[i][fr  :   hrofirst_zero], label='Oracle' + mean_label + dist_type)
        if median:
            ax.plot(x[fr:mhrofirst_zero], scores_hro_med[i][fr  :   mhrofirst_zero], label='Oracle' + median_label + dist_type)

    ax.set_xlabel(x_label)  # Add an x-label to the axes.
    ax.set_ylabel(y_label)  # Add a y-label to the axes.
    ax.set_title(fig_name)  # Add a title to the axes.
    ax.legend();  # Add a legend.
    # plt.show()
    if save_folder is not None:
        plt.savefig(save_folder + "/" + filename + "_oracle.png")

    # for overleaf

    pdf_save_folder = save_folder[:save_folder.find("results_and_plots")] + save_folder[save_folder.find("results_and_plots") + 12:]
    overleaf_load_file = "img/" + save_folder[save_folder.find("results_and_plots") + 12:] + "/" + filename.replace(" ", "_") + "_oracle.pdf"

    os.makedirs(pdf_save_folder, exist_ok=True) 
    plt.savefig(pdf_save_folder + "/" + filename.replace(" ", "_") + "_oracle.pdf", format="pdf", bbox_inches="tight")

    subplot_string = r"""\begin{subfigure}[b]{\textwidth}
    \centering
    \includegraphics[width=\textwidth]{""" + overleaf_load_file + r"""}
    \caption{""" + y_label + r"""}
    \label{fig:""" + fig_name.replace(" ", "_") + "_" + filename.replace(" ", "_") + r"""_oracle}
\end{subfigure}"""

    with open(pdf_save_folder + "/" + filename.replace(" ", "_") + "_oracle.txt", "w") as text_file:
        text_file.write(subplot_string)

    with open(pdf_save_folder + "/../../../../overleaf_accumulator.txt", "a+") as text_file:
        text_file.write(subplot_string)
        text_file.write("\n\n\n")






def plot_dist_scores(res_path, res_name, fig_name, save_folder=None, mean=True, median=True, filename=None, N=9, fr=100,
                hr="_mean_scores_.txt",
                hrm="_median_scores_.txt",
                r="_mean_scores_.txt",
                rm="_median_scores_.txt",
                y_label='best model score', x_label='#iterations'):
    if filename == None:
        filename = fig_name

    scores_hr = []
    scores_hr_med = []
    scores_r = []
    scores_r_med = []

    fr = 15
    expr_range = range(N)
    
    for i in expr_range:
        s_hr, s_hr_med, s_r, s_r_med = load_scores(res_path +'_dist_'+str(i) + res_name, hr, hrm, r, rm)

        scores_hr.append(s_hr)
        scores_hr_med.append(s_hr_med)
        scores_r.append(s_r)
        scores_r_med.append(s_r_med)


    rfirst_zero = 800#np.where(scores_r[0] == scores_r[0][-1])[0][0]
    hrfirst_zero = 800#np.where(scores_hr[0] == scores_hr[0][-1])[0][0]
    mrfirst_zero = 800#np.where(scores_r_med[0] == scores_r_med[0][-1])[0][0]
    mhrfirst_zero = 800#np.where(scores_hr_med[0] == scores_hr_med[0][-1])[0][0]

    first_zero = max(rfirst_zero, hrfirst_zero, mrfirst_zero, mhrfirst_zero)
    x = np.arange(1, first_zero + 1)

    hr_colors = plt.cm.winter(np.linspace(0, 1, 10))  # hr_colors = plt.get_cmap('PuBu')(np.linspace(0, 1, 10))
    r_colors = plt.cm.YlOrBr(np.linspace(0.5, 0.9, 10))

    fig, ax = plt.subplots()

    for i in expr_range:
        if mean:
            ax.plot(x[fr:hrfirst_zero], scores_hr[i][fr  :   hrfirst_zero], label='dist'+str(i), color=hr_colors[i])#hybrid_ransac_mean
            # ax.plot(x[fr:rfirst_zero], scores_r[i][fr  :   rfirst_zero], label='ransac_mean' if i == 0 or i == 9 else None, color=r_colors[i])
        if median:
            ax.plot(x[fr:mhrfirst_zero], scores_hr_med[i][fr  :   mhrfirst_zero], label='dist'+str(i), color=hr_colors[i])#'hybrid_ransac_median'
            # ax.plot(x[fr:mrfirst_zero], scores_r_med[i][fr  :   mrfirst_zero], label='ransac_median' if i == 0 or i == 9 else None, color=r_colors[i])

    ax.set_xlabel(x_label) 
    ax.set_ylabel(y_label)  
    ax.set_title(fig_name) 
    matplotx.line_labels()  
    # plt.show()
    if save_folder is not None:
        plt.savefig(save_folder + "/" + filename + ".png", bbox_inches='tight')



if __name__ == "__main__":
    plot_scores("generated_models\\ShopFacade\\mixinliers\\results\\results_testm2noveoutputy", "testy", None, True, False)
    plot_scores("generated_models\\ShopFacade\\mixinliers\\results\\results_testm2noveoutputy", "testy", None, False, True)