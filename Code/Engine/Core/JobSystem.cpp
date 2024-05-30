#include "JobSystem.hpp"

//-----------------------------------------------------------------------------------------------
JobWorker::JobWorker(JobSystem* jobSystem, int workerThreadID)
	:m_jobSystem(jobSystem)
	,m_workerThreadID(workerThreadID)
{
	m_thread = new std::thread(ThreadMain, m_workerThreadID);
}

//-----------------------------------------------------------------------------------------------
JobWorker::~JobWorker()
{
	m_thread->join();
	delete m_thread;
	m_thread = nullptr;
}

//-----------------------------------------------------------------------------------------------
void JobWorker::ThreadMain(int workerID)
{
	workerID;
	while (!g_theJobSystem->IsQuitting())
	{
		Job* jobToExecute = nullptr;
		jobToExecute = g_theJobSystem->ClaimJobToExecute();
		if (jobToExecute != nullptr)
		{
			jobToExecute->m_jobStatus = JobStatus::CLAIMED;
			jobToExecute->Execute(); // typically very slow
			g_theJobSystem->MoveJobToCompletedList(jobToExecute);
			jobToExecute->m_jobStatus = JobStatus::COMPLETED;
		}
		else
		{
			std::this_thread::sleep_for(std::chrono::microseconds(10));
		}
	}
}

//-----------------------------------------------------------------------------------------------
JobSystem::JobSystem(JobSystemConfig const& config)
	:m_config(config)
{
}

//-----------------------------------------------------------------------------------------------
JobSystem::~JobSystem()
{
	for (int workerIndex = 0; workerIndex < m_workers.size(); workerIndex++)
	{
		if (m_workers[workerIndex] != nullptr)
		{
			delete m_workers[workerIndex];
			m_workers[workerIndex] = nullptr;
		}
	}
}

//-----------------------------------------------------------------------------------------------
void JobSystem::Startup()
{
	int numWorkers = m_config.m_preferredNumberOfWorkers;
	if (numWorkers < 0)
	{
		int numOfCores = std::thread::hardware_concurrency();
		numWorkers = numOfCores - 1;
	}
	CreateWorkers(numWorkers);
}

//-----------------------------------------------------------------------------------------------
void JobSystem::BeginFrame()
{
}

//-----------------------------------------------------------------------------------------------
void JobSystem::EndFrame()
{
}

//-----------------------------------------------------------------------------------------------
void JobSystem::Shutdown()
{
	m_isQuitting = true;
}

//-----------------------------------------------------------------------------------------------
void JobSystem::CreateWorkers(int numWorkerThreads)
{
	for (int workerIndex = 0; workerIndex < numWorkerThreads; workerIndex++)
	{
		JobWorker* newWorkerThread = new JobWorker(this, workerIndex);
		m_workers.push_back(newWorkerThread);
	}
}

//-----------------------------------------------------------------------------------------------
void JobSystem::PostNewJob(Job* job)
{
	m_unclaimedJobsListMutex.lock();
	m_unclaimedJobsList.push_back(job);
	m_unclaimedJobsListMutex.unlock();
}

//-----------------------------------------------------------------------------------------------
bool JobSystem::IsQuitting()
{
	return m_isQuitting;
}

//-----------------------------------------------------------------------------------------------
void JobSystem::MoveJobToCompletedList(Job* job)
{
	m_claimedJobsListMutex.lock();
	m_claimedJobsList.erase(std::remove(m_claimedJobsList.begin(), m_claimedJobsList.end(), job), m_claimedJobsList.end());
	m_claimedJobsListMutex.unlock();

	m_completedJobsListMutex.lock();
	m_completedJobsList.push_back(job);
	m_completedJobsListMutex.unlock();
}

//-----------------------------------------------------------------------------------------------
Job* JobSystem::ClaimJobToExecute()
{
	Job* jobToClaim = nullptr;

	m_unclaimedJobsListMutex.lock();
	if (!m_unclaimedJobsList.empty())
	{
		jobToClaim = m_unclaimedJobsList.front();
		m_unclaimedJobsList.pop_front();

		//move to claimed list
		m_claimedJobsListMutex.lock();
		m_claimedJobsList.push_back(jobToClaim);
		m_claimedJobsListMutex.unlock();
	}
	m_unclaimedJobsListMutex.unlock();

	return jobToClaim;
}

//-----------------------------------------------------------------------------------------------
Job* JobSystem::RetreiveCompletedJob()
{
	Job* completedJob = nullptr;

	m_completedJobsListMutex.lock();
	if (!m_completedJobsList.empty())
	{
		completedJob = m_completedJobsList.front();
		m_completedJobsList.pop_front();
		completedJob->m_jobStatus = JobStatus::RETREIVED;
	}
	m_completedJobsListMutex.unlock();
	return completedJob;
}
