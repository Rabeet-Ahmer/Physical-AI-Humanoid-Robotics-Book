import React, { JSX } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageHeader from '@site/src/components/HomepageHeader';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import HomepageCurriculum from '@site/src/components/HomepageCurriculum';

export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="Master Physical AI, Robotics, and Simulation">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <HomepageCurriculum />
      </main>
    </Layout>
  );
}