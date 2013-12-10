---
layout: default
title: Contribute
---

# Contribute

There are many ways you can contribute to the ROS 2.0 design effort. The most important of which is probably to review the list of articles posted on the [main page]({{ site.baseurl }}/) of the website and familiarize yourself with the discussions on an issue before getting involved. This will help keep everyone on the same page and prevent having to discuss things multiple times.

## Contributing a New Article or Changes to an Existing Article

All of the pages on this website have a sidebar on the right hand side with links to view the source of this page on Github.com and to edit this page in Github.com's on-line editor. In addition to these buttons, all pull requests (open and closed) which involve this file in some way are also listed. Please review the previous pull requests before proposing a change to prevent duplicates and to prevent rehashing issues already addressed.

The "Edit in Github" button will bring you to an on-line editor where you can make simple changes. Github will automatically fork this repository to your github user account, save your on-line changes to a branch there, and open a pull request against our repository on your behalf. This allows us to review and discuss changes to each page before accepting the pull request.

### Submitting a New Article

New articles should be submitted as a new file under the articles folder. You can add a new file using Github.com's web interface using this link:

[https://github.com/ros2/design/new/gh-pages/articles](https://github.com/ros2/design/new/gh-pages/articles)

Otherwise you should manually make a pull request as described below. Either way all articles should be placed under the `articles/` folder.

New articles should follow this outline:

{% raw %}
    ---
    layout: default
    title: Template for New Articles
    abstract:
      This is a multi-line abstract about this article. It should give a good overview about the contents of this article, the reason for writing it, and what the article delivers. The abstract is put in the "front-matter" of the document (YAML between the initial `---`'s) so that it can easily be reused else where in the site.
    ---

    # {{ page.title }}

    ## Abstract

    {{ page.abstract }}

    ## Intro

    The intro should be a slightly more in depth version of the abstract, describing the context of the article in more detail as well as the content, conclusions, and general structure of the article.

    The following sections are options for how to layout the information, they might not all be applicable for all articles, and they might not come in this order or in contiguous blocks, but rather can be brought up at natural points in the discussion.

    ## Problem Space

    The article should describe the purpose of the paper in the context of the other ROS 2.0 design topics and with respect to the ROS 1.x implementation or state. This information should serve to frame the discussion and give the audience a scoped expectation.

    ## Use Cases or User Stories

    One thing you can call out directly in your document are user stories or use cases which should be considered when discussing this topic. The user stories can be collected into one section, or brought up and described through out the document, where ever appropriate.

    ## Experiments and Research

    For any design goals, design decisions, or trade-offs which need to be made and might be unknown or contested, experiments and research should be done and summarized in the article. Like with the use cases, you can either consolidate the results into one section, or bring them up where ever useful throughout the document.

    ## Proposals

    An article should, after discussing the issues and different dimensions of the topic, make solid proposals about how to make design decisions and suggest a general course of action for ROS 2.0. The proposal can be stated as a single monolithic proposal, or many smaller proposals, one for each part of the topic. For example, it might make sense to discuss one dimension of the problem space, make a proposal, and then discuss another dimension. Each proposal should describe a solution or course of action, and should state any trade-offs made, including pros and cons of the proposal.

    ## Alternatives

    If there are alternatives, state them and describe why they were not chosen as the recommended proposal. If an alternative is proposed after the article is written and not accepted, it should be added here and described for posterity.

    ## Open Questions

    It is OK for the article to say that it is not yet clear what should be done (which alternative to pick), though it should strive to make a decision and justify it. Any open issues should be listed here.

    ## Summary

    This section should be in every article, it should summarize the proposals and why they were chosen. This adage applies: "Tell them what you're going to tell them; Tell them; Tell them what you told them." This makes the article, especially if it is long, more accessible to some users.

{% endraw %}

## Manually Making a Pull Request

For more extensive changes or for submitting a new article, you will probably want to fork this repository and clone it to your local machine, edit it there, and then propose a pull request.

### Forking this Repository

First make sure you don't already have a fork of this repository at:

    https://github.com/<your username>/design

If you do not, then browse to this [repository](https://github.com/ros2/design) and click on the "Fork" button:

<img src="{{ site.baseurl }}/img/fork.png"/>

Github.com will now go off and create a fork off this repository onto your user account. Then you can clone your fork of the repository by running this command:

    git clone https://github.com/<your username>/design.git

This will clone this repository onto your machine into a folder called `design`. You can edit these files in this folder using your favorite editor.

### Using Jekyll to Preview Changes

If you change is more extensive or you want to see what your changes will look like on the live site, you can do so by running Jekyll locally. First install Jekyll for your system:

[http://jekyllrb.com/docs/installation/](http://jekyllrb.com/docs/installation/)

Once you have Jekyll installed you should be able to run this command:


    jekyll serve --watch --baseurl=''


The `jekyll server` command will start a web server which you can access at `http://localhost:4000` locally. The `--watch` option will cause the jekyll web server to regenerate pages which are changed each time they are modified. This allows you to make quick edits to the documents and then refresh the web page to get the changes immediately. The `--baseurl=''` option sets the `baseurl` variable for the site's global config, allowing static files like the `css` and `js` files work on your local host.

### Submitting a Pull Request

Once you are satisfied with your changes you can follow the Github.com tutorial on [creating a pull request](https://help.github.com/articles/creating-a-pull-request) against ours, where we can review and discuss your changes before merging.
